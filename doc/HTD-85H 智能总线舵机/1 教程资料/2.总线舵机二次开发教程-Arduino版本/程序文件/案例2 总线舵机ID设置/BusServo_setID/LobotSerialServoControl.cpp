#include <Arduino.h>

int enc_get_utf8_size(uint8_t first_bytes)
{
  uint8_t temp = 0x80;
  int num = 0;
  while (temp & first_bytes)
  {
    num++;
    temp = (temp >> 1);
  }
  return num;
}


int enc_utf8_to_unicode_one(const unsigned char* pInput, unsigned long *Unic)
{
  // b1 表示UTF-8编码的pInput中的高字节, b2 表示次高字节, ...
  char b1, b2, b3, b4, b5, b6;

  *Unic = 0x0; // 把 *Unic 初始化为全零
  int utfbytes = enc_get_utf8_size((uint8_t)*pInput);
  unsigned char *pOutput = (unsigned char *) Unic;

  switch ( utfbytes )
  {
    case 0:
      *pOutput     = *pInput;
      utfbytes    += 1;
      break;
    case 2:
      b1 = *pInput;
      b2 = *(pInput + 1);
      if ( (b2 & 0xE0) != 0x80 )
        return 0;
      *pOutput     = (b1 << 6) + (b2 & 0x3F);
      *(pOutput + 1) = (b1 >> 2) & 0x07;
      break;
    case 3:
      b1 = *pInput;
      b2 = *(pInput + 1);
      b3 = *(pInput + 2);
      if ( ((b2 & 0xC0) != 0x80) || ((b3 & 0xC0) != 0x80) )
        return 0;
      *pOutput     = (b2 << 6) + (b3 & 0x3F);
      *(pOutput + 1) = (b1 << 4) + ((b2 >> 2) & 0x0F);
      break;
    case 4:
      b1 = *pInput;
      b2 = *(pInput + 1);
      b3 = *(pInput + 2);
      b4 = *(pInput + 3);
      if ( ((b2 & 0xC0) != 0x80) || ((b3 & 0xC0) != 0x80)
           || ((b4 & 0xC0) != 0x80) )
        return 0;
      *pOutput     = (b3 << 6) + (b4 & 0x3F);
      *(pOutput + 1) = (b2 << 4) + ((b3 >> 2) & 0x0F);
      *(pOutput + 2) = ((b1 << 2) & 0x1C)  + ((b2 >> 4) & 0x03);
      break;
    case 5:
      b1 = *pInput;
      b2 = *(pInput + 1);
      b3 = *(pInput + 2);
      b4 = *(pInput + 3);
      b5 = *(pInput + 4);
      if ( ((b2 & 0xC0) != 0x80) || ((b3 & 0xC0) != 0x80)
           || ((b4 & 0xC0) != 0x80) || ((b5 & 0xC0) != 0x80) )
        return 0;
      *pOutput     = (b4 << 6) + (b5 & 0x3F);
      *(pOutput + 1) = (b3 << 4) + ((b4 >> 2) & 0x0F);
      *(pOutput + 2) = (b2 << 2) + ((b3 >> 4) & 0x03);
      *(pOutput + 3) = (b1 << 6);
      break;
    case 6:
      b1 = *pInput;
      b2 = *(pInput + 1);
      b3 = *(pInput + 2);
      b4 = *(pInput + 3);
      b5 = *(pInput + 4);
      b6 = *(pInput + 5);
      if ( ((b2 & 0xC0) != 0x80) || ((b3 & 0xC0) != 0x80)
           || ((b4 & 0xC0) != 0x80) || ((b5 & 0xC0) != 0x80)
           || ((b6 & 0xC0) != 0x80) )
        return 0;
      *pOutput     = (b5 << 6) + (b6 & 0x3F);
      *(pOutput + 1) = (b5 << 4) + ((b6 >> 2) & 0x0F);
      *(pOutput + 2) = (b3 << 2) + ((b4 >> 4) & 0x03);
      *(pOutput + 3) = ((b1 << 6) & 0x40) + (b2 & 0x3F);
      break;
    default:
      return 0;
      break;
  }

  return utfbytes;
}

int enc_utf8_to_unicode_string(const unsigned char* pInput, unsigned long *Unic)
{
  uint32_t len = strlen(pInput);
  uint32_t i = 0;
  unsigned long buf;
  int word_count = 0;
  uint16_t *unic_buf = (uint16_t*)Unic; /*unicode 是4个字节，这里强行只使用两个字节*/
  while(i < len) {
    int word_len = enc_utf8_to_unicode_one(pInput, &buf);
    pInput += word_len;
    i += word_len;
    *unic_buf++ = (uint16_t)buf; /* 强行转成两个字节， 中文在这个范围内。 */
    ++word_count;
  }
  return word_count;
}
