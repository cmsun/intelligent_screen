#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
香橙派 5 / RK3588 + 三路 HC-SR04P 超声波测距（Ubuntu 22.04 / 24.04）

三路接线（RK3588 每组 gpiochip 32 条线：A=0~7, B=8~15, C=16~23, D=24~31，
偏移 = 组号×8 + 序号；GPIO1_* -> gpiochip1，GPIO4_* -> gpiochip4）
    左  Trig GPIO1_A2 (gpiochip1:2)    Echo GPIO1_A4 (gpiochip1:4)
    中  Trig GPIO1_A6 (gpiochip1:6)    Echo GPIO1_D7 (gpiochip1:31)
    右  Trig GPIO4_A7 (gpiochip4:7)    Echo GPIO4_A6 (gpiochip4:6)
    三个模块 VCC -> 3.3V（推荐，Echo 天然 3.3V）；若接 5V，Echo 必须经
        1kΩ + 2kΩ 分压到 3.3V，RK3588 的 GPIO 不耐 5V！
    三个模块 GND 必须与板子共地，注意 3.3V 供电电流余量。

跑之前先核准：
    sudo apt install gpiod python3-libgpiod -y
    gpiodetect
    gpiofind "GPIO1_A2"       # 应输出 gpiochip1 2
    gpioinfo gpiochip1        # 看这 6 条线有没有被别的驱动占用
    编号不同用 --chip1 / --chip4 覆盖；要改引脚本身编辑 build_configs()

权限（别一直用 sudo 跑业务）：
    sudo usermod -aG gpio $USER
    /etc/udev/rules.d/99-gpio.rules 写一行：
      SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add|change", RUN+="/bin/sh -c 'chown root:gpio /dev/%k && chmod 660 /dev/%k'"

libgpiod 版本：
    Ubuntu 22.04 = 1.6.x（LINE_REQ_* / event_wait / event_read）
    Ubuntu 24.04 = 2.x  （request_lines / LineSettings / EdgeEvent）
    本文件运行时自动检测，两套 API 都支持；时间戳都来自内核中断，精度一致。

三路工作方式：一轮内逐路触发，等本路回波结束（或超时）再触发下一路，
避免多个 40kHz 探头同时发射串扰；每路自身触发周期仍 >= 60 ms。

用法：
    python3 hc_sr04p_opi5.py
    python3 hc_sr04p_opi5.py --sensor left --raw
    python3 hc_sr04p_opi5.py --stats 100 --sensor right
"""

import argparse
import os
import statistics
import sys
import time

import gpiod

try:  # libgpiod 2.x（Ubuntu 24.04+）
    from gpiod.line import Bias, Direction, Edge, Value
    GPIOD_V2 = hasattr(gpiod, "request_lines")
except ImportError:  # libgpiod 1.6.x（Ubuntu 22.04）
    Bias = Direction = Edge = Value = None
    GPIOD_V2 = False


# ---------------------------------------------------------------- 配置
CHIP1 = 1                    # GPIO1_* 所在 gpiochip 编号
CHIP4 = 4                    # GPIO4_* 所在 gpiochip 编号
TEMP_C = 25.0                # 换成 DS18B20 / SHT30 / BMP280 的实测气温
SAMPLES = 9                  # 每轮每路采样数（奇数，便于取中值）
INTERVAL_MS = 80.0           # 轮间隔 ms，模块规格要求 >= 60
GAP_MS = 20.0                # 同一轮内相邻两路触发间隔 ms（抑制串扰）
TIMEOUT_MS = 40.0            # 单次回波超时 ms，约对应 6.8 m
TRIG_PULSE_S = 3e-5          # 触发高电平，规格只要 >= 10µs


def build_configs(chip1=CHIP1, chip4=CHIP4):
    """三路引脚表：offset = 组号(A=0,B=1,C=2,D=3) * 8 + 序号。"""
    return [
        dict(label="左", key="left",   trig_name="GPIO1_A2", echo_name="GPIO1_A4",
             trig_chip=chip1, trig_ofs=2, echo_chip=chip1, echo_ofs=4),
        dict(label="中", key="center", trig_name="GPIO1_A6", echo_name="GPIO1_D7",
             trig_chip=chip1, trig_ofs=6, echo_chip=chip1, echo_ofs=31),
        dict(label="右", key="right",  trig_name="GPIO4_A7", echo_name="GPIO4_A6",
             trig_chip=chip4, trig_ofs=7, echo_chip=chip4, echo_ofs=6),
    ]


def sensor_index(name):
    if name in ("left", "l", "左", "0"):
        return 0
    if name in ("center", "middle", "c", "m", "中", "1"):
        return 1
    if name in ("right", "r", "右", "2"):
        return 2
    raise ValueError("--sensor 只能是 left|center|right / 左|中|右 / 0|1|2")


def speed_cm_us(temp_c: float) -> float:
    """声速：v = 331.3 + 0.606 * θ  (m/s) -> cm/µs"""
    return (331.3 + 0.606 * temp_c) / 1e4


def pin_to_big_core() -> None:
    """绑到 A76 大核并提优先级，降低用户态被抢占的概率（非必须，无害）"""
    try:
        os.sched_setaffinity(0, {4})
    except (OSError, AttributeError):
        pass
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(80))
    except (PermissionError, AttributeError, OSError):
        pass


def robust_width_ns(raw):
    """中值 -> 剔除 >5% 偏差 -> 均值，返回 ns；有效样本不足返回 None。"""
    if len(raw) < 3:
        return None
    med = statistics.median(raw)
    good = [x for x in raw if abs(x - med) < 0.05 * med]
    return statistics.fmean(good or [med])


# ---------------------------------------------------------------- GPIO 封装
class GpioLine:
    """libgpiod 1.6 / 2.x 的最小统一封装：输出电平、读电平、双边沿事件。"""

    def __init__(self, chip_num, offset, consumer, output=False):
        self.offset = offset
        self.v2 = GPIOD_V2
        self.req = self.chip = self.line = None
        path = f"/dev/gpiochip{chip_num}"
        if self.v2:
            if output:
                settings = gpiod.LineSettings(
                    direction=Direction.OUTPUT, output_value=Value.INACTIVE)
            else:
                settings = gpiod.LineSettings(
                    direction=Direction.INPUT, edge_detection=Edge.BOTH,
                    bias=Bias.DISABLED)
            self.req = gpiod.request_lines(
                path, consumer=consumer, config={offset: settings})
        else:
            chip_cls = getattr(gpiod, "Chip", None) or gpiod.chip
            self.chip = chip_cls(path)
            self.line = self.chip.get_line(offset)
            if output:
                self.line.request(consumer=consumer,
                                  type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
            else:
                bias_disable = getattr(gpiod, "LINE_REQ_FLAG_BIAS_DISABLE", 0)
                self.line.request(consumer=consumer,
                                  type=gpiod.LINE_REQ_EV_BOTH_EDGES,
                                  flags=bias_disable)

    def set(self, high: bool) -> None:
        if self.v2:
            self.req.set_value(self.offset,
                               Value.ACTIVE if high else Value.INACTIVE)
        else:
            self.line.set_value(1 if high else 0)

    def get_value(self) -> int:
        if self.v2:
            return 1 if self.req.get_value(self.offset) == Value.ACTIVE else 0
        return self.line.get_value()

    def wait_edge(self, timeout_s: float) -> bool:
        """是否有边沿事件可读。"""
        if self.v2:
            return self.req.wait_edge_events(timeout_s)
        return self.line.event_wait(sec=timeout_s)

    def read_events(self):
        """取出全部事件，统一为 [(is_rising, timestamp_ns), ...]。"""
        if self.v2:
            rising = gpiod.EdgeEvent.Type.RISING_EDGE
            return [(ev.event_type == rising, ev.timestamp_ns)
                    for ev in self.req.read_edge_events()]
        rising = getattr(gpiod.LineEvent, "RISING_EDGE", 1)
        out = []
        while self.line.event_wait(sec=0):
            ev = self.line.event_read()
            out.append((ev.type == rising, ev.timestamp_ns))
        return out

    def drain(self) -> None:
        """丢掉上一轮残留的边沿事件。"""
        while self.wait_edge(0):
            self.read_events()

    def release(self) -> None:
        if self.v2:
            if self.req is not None:
                self.req.release()
                self.req = None
        else:
            if self.line is not None:
                self.line.release()
                self.line = None
            if self.chip is not None:
                try:
                    self.chip.close()
                except AttributeError:
                    pass
                self.chip = None


# ---------------------------------------------------------------- 单路传感器
class HcSr04p:
    def __init__(self, cfg, timeout_s=TIMEOUT_MS / 1000.0, temp_c=TEMP_C):
        self.cfg = cfg
        self.timeout_s = timeout_s
        self.temp_c = temp_c
        self.trig = GpioLine(cfg["trig_chip"], cfg["trig_ofs"],
                             f"sr04-{cfg['key']}-trig", output=True)
        try:
            self.echo = GpioLine(cfg["echo_chip"], cfg["echo_ofs"],
                                 f"sr04-{cfg['key']}-echo", output=False)
        except Exception:
            self.trig.release()
            raise
        time.sleep(0.02)                  # 等模块上电稳定
        self.echo.drain()

    def close(self) -> None:
        self.echo.release()
        self.trig.release()

    def _wait_echo_low(self) -> None:
        """Echo 若仍为高，新上升沿不会到来，必须先等它变低。"""
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            try:
                if not self.echo.get_value():
                    return
            except OSError:
                return
            time.sleep(0.002)

    def pulse_ns(self):
        """触发一次，返回 Echo 高电平宽度（ns）；无有效回波返回 None。"""
        self.echo.drain()
        self._wait_echo_low()

        self.trig.set(True)
        time.sleep(TRIG_PULSE_S)
        self.trig.set(False)

        t_rise = None
        t0 = time.monotonic_ns()
        while True:
            remain = self.timeout_s - (time.monotonic_ns() - t0) / 1e9
            if remain <= 0:
                return None
            if not self.echo.wait_edge(min(remain, 0.02)):
                continue
            for is_rising, ts in self.echo.read_events():
                # timestamp_ns 由内核在 GPIO 中断中采集，两个沿来自同一时钟源，
                # 采集延迟相互抵消，比用户态轮询电平精确得多。
                if is_rising:
                    t_rise = ts
                elif t_rise is not None and ts > t_rise:
                    return ts - t_rise


# ---------------------------------------------------------------- CLI / 主流程
def parse_args():
    p = argparse.ArgumentParser(
        description="RK3588 三路 HC-SR04P 测距",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--sensor", default=None, metavar="NAME",
                   help="只跑一路：left|center|right / 左|中|右 / 0|1|2（默认三路全跑）")
    p.add_argument("--chip1", type=int, default=CHIP1, help="GPIO1_* 所在 gpiochip 编号")
    p.add_argument("--chip4", type=int, default=CHIP4, help="GPIO4_* 所在 gpiochip 编号")
    p.add_argument("--temp", type=float, default=TEMP_C, help="环境气温 °C")
    p.add_argument("--samples", type=int, default=SAMPLES, help="每轮每路采样次数（奇数）")
    p.add_argument("--interval", type=float, default=INTERVAL_MS, metavar="MS",
                   help="轮与轮之间的间隔 ms，必须 >= 60")
    p.add_argument("--gap", type=float, default=GAP_MS, metavar="MS",
                   help="同一轮内相邻两路触发间隔 ms")
    p.add_argument("--timeout", type=float, default=TIMEOUT_MS, metavar="MS",
                   help="单次回波超时 ms")
    p.add_argument("--raw", action="store_true", help="额外打印原始回波宽度（µs）")
    p.add_argument("--count", type=int, default=0, help="打印 N 轮后退出，0 = 一直跑")
    p.add_argument("--stats", type=int, default=0, metavar="N",
                   help="抖动诊断：逐路采 N 次后打印统计并退出")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if args.samples % 2 == 0:
        args.samples += 1
    args.samples = max(3, args.samples)
    args.interval = max(60.0, args.interval)
    args.gap = max(0.0, args.gap)

    try:
        sel = sensor_index(args.sensor) if args.sensor is not None else None
    except ValueError as e:
        print(e, file=sys.stderr)
        return 2

    configs = build_configs(args.chip1, args.chip4)
    active = [sel] if sel is not None else [0, 1, 2]

    pin_to_big_core()

    sensors = {}
    try:
        for i in active:
            c = configs[i]
            try:
                sensors[i] = HcSr04p(c, timeout_s=args.timeout / 1000.0,
                                     temp_c=args.temp)
            except OSError as e:
                print(f"初始化失败 [{c['label']}]："
                      f"Trig {c['trig_name']}=gpiochip{c['trig_chip']}:{c['trig_ofs']}，"
                      f"Echo {c['echo_name']}=gpiochip{c['echo_chip']}:{c['echo_ofs']}\n"
                      f"  {e}\n  排查：gpiodetect / gpioinfo / 设备树 pinmux 是否占用该引脚",
                      file=sys.stderr)
                return 1

        print("已就绪：" + "  ".join(
            f"[{configs[i]['label']} Trig {configs[i]['trig_name']}="
            f"gpiochip{configs[i]['trig_chip']}:{configs[i]['trig_ofs']} / "
            f"Echo {configs[i]['echo_name']}="
            f"gpiochip{configs[i]['echo_chip']}:{configs[i]['echo_ofs']}]"
            for i in active))
        print(f"libgpiod {'2.x' if GPIOD_V2 else '1.6.x'}，每轮每路 {args.samples} 次采样，"
              f"轮间隔 {args.interval:.0f} ms，路间隔 {args.gap:.0f} ms，"
              f"气温 {args.temp:.1f} °C\n")

        v = speed_cm_us(args.temp)

        # ---- 抖动诊断（逐路统计）
        if args.stats:
            ok = 0
            for i in active:
                c = configs[i]
                raw = []
                print(f"[{c['label']}] 采集中（{args.stats} 次，间隔 {args.interval:.0f} ms）...")
                for _ in range(args.stats):
                    w = sensors[i].pulse_ns()
                    if w:
                        raw.append(w)
                    time.sleep(args.interval / 1000.0)
                if not raw:
                    print(f"[{c['label']}] 一次有效回波都没有：检查目标、供电、共地、分压。",
                          file=sys.stderr)
                    continue
                us = [x * 1e-3 for x in raw]
                cm = [x * v / 2.0 for x in us]
                sd = statistics.stdev(us) if len(us) > 1 else 0.0
                print(f"\n[{c['label']}] 有效样本 {len(raw)} / {args.stats}")
                print(f"  回波宽度 中值 {statistics.median(us):.2f} µs  "
                      f"均值 {statistics.fmean(us):.2f} µs  标准差 {sd:.3f} µs")
                print(f"  折算距离 中值 {statistics.median(cm):.3f} cm  "
                      f"最小 {min(cm):.3f} cm  最大 {max(cm):.3f} cm\n")
                ok += 1
            print("判读：标准差 < 2 mm 已达标；野值偏大可加大 --gap 抑制三路串扰。")
            return 0 if ok else 1

        # ---- 连续测量：一轮内逐路触发，路间隔 + 轮间隔保证每路 >= 60ms
        printed = 0
        while True:
            raw = {i: [] for i in active}
            for r in range(args.samples):
                for j, i in enumerate(active):
                    w = sensors[i].pulse_ns()
                    if w:
                        raw[i].append(w)
                    if j + 1 < len(active):
                        time.sleep(args.gap / 1000.0)
                if r + 1 < args.samples:
                    time.sleep(args.interval / 1000.0)

            cells = []
            for i in active:
                width = robust_width_ns(raw[i])
                if width is None:
                    cells.append(f"[{configs[i]['label']}] --  (无回波)")
                    continue
                cm = width * 1e-3 * v / 2.0
                if args.raw:
                    cells.append(f"[{configs[i]['label']}] {cm:8.3f} cm "
                                 f"({width * 1e-3:8.2f} µs, {len(raw[i])}/{args.samples})")
                else:
                    cells.append(f"[{configs[i]['label']}] {cm:8.3f} cm")
            print("   ".join(cells), flush=True)

            printed += 1
            if args.count and printed >= args.count:
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        for s in sensors.values():
            s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
