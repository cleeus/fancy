
#include "tm1637.h"
#include "tm1637_config.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#if _TM1637_FREERTOS == 0
#define tm1637_delay_ms(x)  HAL_Delay(x)
#else
#include "cmsis_os.h"
#define tm1637_delay_ms(x)  osDelay(x)
#endif

#define TM1637_COMM1    0x40
#define TM1637_COMM2    0xC0
#define TM1637_COMM3    0x80


/*
 * standard 7seg bit/segment positions:
 *                           5 (top) = 0x20
 *                         +---+
 * 4 (left top)    = 0x10  |   | 0 (right top) = 0x01
 *                         +---+ 6 (middle line) = 0x40
 * 3 (left bottom) = 0x08  |   | 1 (right bottom) = 0x02
 *                         +---+ . 7 (dot) = 0x80
 *                           2 (bottom) = 0x04
 */



static const uint8_t _tm1637_digit[] = {
		[0] = TM1637_LEFT_BOTTOM | TM1637_LEFT_TOP | TM1637_TOP | TM1637_RIGHT_TOP | TM1637_RIGHT_BOTTOM | TM1637_BOTTOM,
		[1] = TM1637_RIGHT_TOP | TM1637_RIGHT_BOTTOM,
		[2] = TM1637_TOP | TM1637_RIGHT_TOP | TM1637_MIDDLE | TM1637_LEFT_BOTTOM | TM1637_BOTTOM,
		[3] = TM1637_TOP | TM1637_RIGHT_TOP | TM1637_MIDDLE | TM1637_RIGHT_BOTTOM | TM1637_BOTTOM,
		[4] = TM1637_LEFT_TOP | TM1637_MIDDLE | TM1637_RIGHT_TOP | TM1637_RIGHT_BOTTOM,
		[5] = TM1637_TOP | TM1637_LEFT_TOP | TM1637_MIDDLE | TM1637_RIGHT_BOTTOM | TM1637_BOTTOM,
		[6] = TM1637_TOP | TM1637_LEFT_TOP | TM1637_LEFT_BOTTOM | TM1637_BOTTOM | TM1637_RIGHT_BOTTOM | TM1637_MIDDLE,
		[7] = TM1637_TOP | TM1637_RIGHT_TOP | TM1637_RIGHT_BOTTOM,
		[8] = TM1637_TOP | TM1637_MIDDLE | TM1637_BOTTOM | TM1637_LEFT_TOP | TM1637_LEFT_BOTTOM | TM1637_RIGHT_TOP | TM1637_RIGHT_BOTTOM,
		[9] = TM1637_RIGHT_BOTTOM | TM1637_RIGHT_TOP | TM1637_TOP | TM1637_LEFT_TOP | TM1637_MIDDLE
};
static const uint8_t _tm1637_on[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t _tm1637_off[6] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
static const uint8_t _tm1637_minus = TM1637_MIDDLE;
static const uint8_t _tm1637_dot = TM1637_DOT;
static const uint8_t _tm1637_degrees = TM1637_TOP | TM1637_RIGHT_TOP | TM1637_MIDDLE | TM1637_LEFT_TOP;
static const uint8_t _tm1637_apostrophe = TM1637_RIGHT_TOP;
static const uint8_t _tm1637_uscore = TM1637_BOTTOM;
static const uint8_t _tm1637_chars_lower['z'-'a'] = {
		['b'-'a'] = TM1637_LEFT_TOP | TM1637_LEFT_BOTTOM | TM1637_MIDDLE | TM1637_RIGHT_BOTTOM | TM1637_BOTTOM,
		['c'-'a'] = TM1637_BOTTOM | TM1637_LEFT_BOTTOM | TM1637_MIDDLE,
		['d'-'a'] = TM1637_RIGHT_TOP | TM1637_RIGHT_BOTTOM | TM1637_BOTTOM | TM1637_LEFT_BOTTOM | TM1637_MIDDLE,
		['i'-'a'] = TM1637_LEFT_BOTTOM,
		['o'-'a'] = TM1637_MIDDLE | TM1637_RIGHT_BOTTOM | TM1637_BOTTOM | TM1637_LEFT_BOTTOM,
		['t'-'a'] = TM1637_LEFT_TOP | TM1637_MIDDLE | TM1637_LEFT_BOTTOM | TM1637_BOTTOM,
		['u'-'a'] = TM1637_LEFT_BOTTOM | TM1637_BOTTOM | TM1637_RIGHT_BOTTOM,
};
static const uint8_t _tm1637_chars_UPPER['Z'-'A'] = {
		['A'-'A'] = TM1637_LEFT_BOTTOM | TM1637_LEFT_TOP | TM1637_TOP | TM1637_RIGHT_TOP | TM1637_RIGHT_BOTTOM | TM1637_MIDDLE,
		['C'-'A'] = TM1637_BOTTOM | TM1637_LEFT_BOTTOM | TM1637_LEFT_TOP | TM1637_TOP,
		['E'-'A'] = TM1637_BOTTOM | TM1637_LEFT_BOTTOM | TM1637_MIDDLE | TM1637_LEFT_TOP | TM1637_TOP,
		['F'-'A'] = TM1637_LEFT_BOTTOM | TM1637_MIDDLE | TM1637_LEFT_TOP | TM1637_TOP,
		['H'-'A'] = TM1637_LEFT_BOTTOM | TM1637_LEFT_TOP | TM1637_MIDDLE | TM1637_RIGHT_BOTTOM | TM1637_RIGHT_TOP,
		['I'-'A'] = TM1637_LEFT_BOTTOM | TM1637_LEFT_TOP,
		['J'-'A'] = TM1637_RIGHT_TOP | TM1637_RIGHT_BOTTOM | TM1637_BOTTOM,
		['L'-'A'] = TM1637_LEFT_TOP | TM1637_LEFT_BOTTOM | TM1637_BOTTOM,
		['P'-'A'] = TM1637_LEFT_TOP | TM1637_LEFT_BOTTOM | TM1637_TOP | TM1637_RIGHT_TOP | TM1637_MIDDLE,
		['U'-'A'] = TM1637_LEFT_TOP | TM1637_LEFT_BOTTOM | TM1637_BOTTOM | TM1637_RIGHT_TOP | TM1637_RIGHT_BOTTOM,
};


//#######################################################################################################################
static void tm1637_delay_us(uint8_t delay)
{
  while (delay > 0)
  {
    delay--;
    //__nop();__nop();__nop();__nop();
    for (int i = 0; i < 4; i++) {
    	__asm__ __volatile__("nop\n\t":::"memory");
    }
  }
}
//#######################################################################################################################
static void tm1637_start(tm1637_t *tm1637)
{
  HAL_GPIO_WritePin(tm1637->gpio_dat, tm1637->pin_dat, tm1637->gpio_reset);
  tm1637_delay_us(_TM1637_BIT_DELAY);
}
//#######################################################################################################################
static void tm1637_stop(tm1637_t *tm1637)
{
  HAL_GPIO_WritePin(tm1637->gpio_dat, tm1637->pin_dat, tm1637->gpio_reset);
  tm1637_delay_us(_TM1637_BIT_DELAY);
  HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, tm1637->gpio_set);
  tm1637_delay_us(_TM1637_BIT_DELAY);
  HAL_GPIO_WritePin(tm1637->gpio_dat, tm1637->pin_dat, tm1637->gpio_set);
  tm1637_delay_us(_TM1637_BIT_DELAY);
}
//#######################################################################################################################
static uint8_t tm1637_write_byte(tm1637_t *tm1637, uint8_t data)
{
  //  write 8 bit data
  for (uint8_t i = 0; i < 8; i++)
  {
    HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, tm1637->gpio_reset);
    tm1637_delay_us(_TM1637_BIT_DELAY);
    if (data & 0x01)
      HAL_GPIO_WritePin(tm1637->gpio_dat, tm1637->pin_dat, tm1637->gpio_set);
    else
      HAL_GPIO_WritePin(tm1637->gpio_dat, tm1637->pin_dat, tm1637->gpio_reset);
    tm1637_delay_us(_TM1637_BIT_DELAY);
    HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, tm1637->gpio_set);
    tm1637_delay_us(_TM1637_BIT_DELAY);
    data = data >> 1;
  }
  // wait for acknowledge
  HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, tm1637->gpio_reset);
  HAL_GPIO_WritePin(tm1637->gpio_dat, tm1637->pin_dat, tm1637->gpio_set);
  tm1637_delay_us(_TM1637_BIT_DELAY);
  HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, tm1637->gpio_set);
  tm1637_delay_us(_TM1637_BIT_DELAY);
  uint8_t ack = HAL_GPIO_ReadPin(tm1637->gpio_dat, tm1637->pin_dat);
  if (ack == 0)
    HAL_GPIO_WritePin(tm1637->gpio_dat, tm1637->pin_dat, tm1637->gpio_reset);
  tm1637_delay_us(_TM1637_BIT_DELAY);
  HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, tm1637->gpio_reset);
  tm1637_delay_us(_TM1637_BIT_DELAY);
  return ack;
}
//#######################################################################################################################
static void tm1637_lock(tm1637_t *tm1637)
{
  while (tm1637->lock == 1)
    tm1637_delay_ms(1);
  tm1637->lock = 1;  
}
//#######################################################################################################################
static void tm1637_unlock(tm1637_t *tm1637)
{
  tm1637->lock = 0;  
}
//#######################################################################################################################
void tm1637_init(tm1637_t *tm1637, GPIO_TypeDef *gpio_clk, uint16_t pin_clk, GPIO_TypeDef *gpio_dat, uint16_t pin_dat, TM1637_Polarity polarity)
{
  memset(tm1637, 0, sizeof(tm1637_t)); 
  //  set max brightess
  tm1637_brightness(tm1637, 7);  
  tm1637_lock(tm1637);
  //  init gpio
  tm1637->gpio_clk = gpio_clk;
  tm1637->pin_clk = pin_clk;
  tm1637->gpio_dat = gpio_dat;
  tm1637->pin_dat = pin_dat;

  GPIO_InitTypeDef g = {0};
  if(polarity==TM1637_POLARITY_NORMAL_OD) {
  	tm1637->gpio_reset = GPIO_PIN_RESET;
  	tm1637->gpio_set = GPIO_PIN_SET;
    g.Mode = GPIO_MODE_OUTPUT_OD;
  } else if(polarity == TM1637_POLARITY_INV_PP) {
  	tm1637->gpio_reset = GPIO_PIN_SET;
  	tm1637->gpio_set = GPIO_PIN_RESET;
    g.Mode = GPIO_MODE_OUTPUT_PP;
  }
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Pin = pin_clk;
  HAL_GPIO_Init(gpio_clk, &g);
  g.Pin = pin_dat;
  HAL_GPIO_Init(gpio_dat, &g);    
  tm1637_unlock(tm1637);
}
//#######################################################################################################################
void tm1637_brightness(tm1637_t *tm1637, uint8_t brightness_0_to_7)
{
  tm1637_lock(tm1637);
  tm1637->brightness = (brightness_0_to_7 & 0x7) | 0x08;
  tm1637_unlock(tm1637);
}
//#######################################################################################################################
static void tm1637_write_raw(tm1637_t *tm1637, const uint8_t *raw, uint8_t length, uint8_t pos)
{
  if (pos > 5)
    return;
  if (length > 6)
    length = 6;
  // write COMM1
  tm1637_start(tm1637);
  tm1637_write_byte(tm1637, TM1637_COMM1);
  tm1637_stop(tm1637);
  // write COMM2 + first digit address
  tm1637_start(tm1637);
  tm1637_write_byte(tm1637, TM1637_COMM2 + (pos & 0x03));
  // write the data bytes
  for (uint8_t k=0; k < length; k++)
    tm1637_write_byte(tm1637, raw[k]);
  tm1637_stop(tm1637);
  // write COMM3 + brightness
  tm1637_start(tm1637);
  tm1637_write_byte(tm1637, TM1637_COMM3 + tm1637->brightness);
  tm1637_stop(tm1637);
}
//#######################################################################################################################
void tm1637_write_segment(tm1637_t *tm1637, const uint8_t *segments, uint8_t length, uint8_t pos)
{
  tm1637_lock(tm1637);
  tm1637_write_raw(tm1637, segments, length, pos);
  tm1637_unlock(tm1637);  
}
//#######################################################################################################################
static void tm1637_i32toa_n(char *s, const size_t len, int32_t val, const int width) {
	char buf[16] = {0};
	int i = 0;

	//check for negativity and special case INT32_MIN
	const bool negative = val < 0;
	if(negative) {
		if(val == INT32_MIN) {
			strncpy(buf, "-2147483648", sizeof(buf)-1);
			goto tm1637_i32toa_n_OUTPUT;
		}
		//convert to positive, prepend minus at the end
		val = -val;
	}

	//base10 conversion loop
	{
		for(i=sizeof(buf)-2; val && i  ; i--) {
			buf[i] = "0123456789"[val % 10];
			val /= 10;
		}
		//prepend zeroes
		for(; (sizeof(buf) - i - 1) <= width && i ; i--) {
			buf[i] = '0';
		}
		//prepend minus
		if(negative) {
			buf[i] = '-';
		} else {
			i++;
		}
	}

tm1637_i32toa_n_OUTPUT:
  strncpy(s, &buf[i], len);
  s[len-1] = '\0';
}

static void tm1637_ato7seg(uint8_t buffer[6], const char *str, const uint8_t length) {
  uint8_t index = 0;
  for (uint8_t i=0; i<length && index<6; i++) {
  	switch(str[i]) {
  		case '\0':
  			//fill output buffer with 0:
  			for(;index<6;index++) {
  				buffer[index]=0;
  			}
  			//outer loop abort condition index==6 now reached
  			break;
  		case '-':
  			buffer[index] = _tm1637_minus;
  			index++;
  			break;
  		case '0':
  		case '1':
  		case '2':
  		case '3':
  		case '4':
  		case '5':
  		case '6':
  		case '7':
  		case '8':
  		case '9':
        buffer[index] = _tm1637_digit[str[i] - '0'];
        index++;
        break;
  		case 'b':
  		case 'c':
  		case 'd':
  		case 'i':
  		case 'o':
  		case 't':
  		case 'u':
  			buffer[index] = _tm1637_chars_lower[str[i]-'a'];
  			index++;
  			break;
  		case 'A':
  		case 'C':
  		case 'E':
  		case 'F':
  		case 'H':
  		case 'I':
  		case 'J':
  		case 'L':
  		case 'P':
  		case 'U':
  			buffer[index] = _tm1637_chars_UPPER[str[i]-'A'];
  			index++;
  			break;
  		case '.':
        if (index > 0)
          buffer[index - 1] |= _tm1637_dot;
        break;
  		case '\'':
  			buffer[index] = _tm1637_apostrophe;
  			index++;
  			break;
  		case (char)(0xBA): //degrees sign in windows CP-1252
  			buffer[index] = _tm1637_degrees;
  			index++;
  			break;
  		case '_':
  			buffer[index] = _tm1637_uscore;
  			index++;
  			break;
  		default:
  			buffer[index] = 0;
  			break;
  	}
  }
}

void tm1637_write_int(tm1637_t *tm1637, int32_t digit, uint8_t pos)
{
  tm1637_lock(tm1637);
  char str[7];
  uint8_t buffer[6] = {0};
  tm1637_i32toa_n(str, sizeof(str), digit, 1);
  tm1637_ato7seg(buffer, str, sizeof(str));
  tm1637_write_raw(tm1637, buffer, 6, pos);
  tm1637_unlock(tm1637);
}


void tm1637_write_fractional(tm1637_t *tm1637, char prefix, float digit, uint8_t floating_digit, uint8_t pos)
{
  tm1637_lock(tm1637);
  char str[32];
  uint8_t buffer[6] = {0};
  const int16_t  digit_int   =  digit;
  uint16_t fract_factor = 1;
  for(int i=0;i<floating_digit;i++) {
  	fract_factor *= 10;
  }
  const uint16_t digit_fract = (digit - digit_int) * fract_factor;

  if(prefix != '\0') {
  	str[0] = prefix;
  	str[1] = '\0';
    tm1637_i32toa_n(str+1, sizeof(str)-1, digit_int, 1);
  } else {
    tm1637_i32toa_n(str, sizeof(str), digit_int, 1);
  }
  if(floating_digit > 0) {
  	strncat(str, ".", sizeof(str)-1);
  	const size_t len = strnlen(str, sizeof(str));
  	tm1637_i32toa_n(&str[len], sizeof(str)-len, digit_fract, 1);
  }

  if (tm1637->show_zero == false)
  {
    for (int8_t i = strlen(str) - 1; i > 0; i--)
    {
      if (str[i] == '0')
        str[i] = 0;
      else
        break;
    }
  }

  tm1637_ato7seg(buffer, str, sizeof(str));

  tm1637_write_raw(tm1637, buffer, 6, pos);
  tm1637_unlock(tm1637);
}

void tm1637_write_str(tm1637_t *tm1637, const char *str, uint8_t length, uint8_t pos) {
  tm1637_lock(tm1637);
  uint8_t buffer[6] = {0};

  tm1637_ato7seg(buffer, str, length);

  tm1637_write_raw(tm1637, buffer, 6, pos);
  tm1637_unlock(tm1637);
}


#if 0
//#######################################################################################################################
void tm1637_write_float(tm1637_t *tm1637, float digit, uint8_t floating_digit, uint8_t pos)
{
  tm1637_lock(tm1637);
  char str[8];
  uint8_t buffer[6] = {0};
  if (floating_digit >6)
    floating_digit = 6;
  switch (floating_digit)
  {
    case 0:
      snprintf(str, sizeof(str) , "%.0f", digit);
    break;
    case 1:
      snprintf(str, sizeof(str) , "%.1f", digit);
    break;
    case 2:
      snprintf(str, sizeof(str) , "%.2f", digit);
    break;
    case 3:
      snprintf(str, sizeof(str) , "%.3f", digit);
    break;
    case 4:
      snprintf(str, sizeof(str) , "%.4f", digit);
    break;
    case 5:
      snprintf(str, sizeof(str) , "%.5f", digit);
    break;
    case 6:
      snprintf(str, sizeof(str) , "%.6f", digit);
    break;
  } 
  if (tm1637->show_zero == false)
  {
    for (int8_t i = strlen(str) - 1; i > 0; i--)
    {
      if (str[i] == '0')
        str[i] = 0;
      else
        break;            
    }
  }
  uint8_t index = 0;  
  for (uint8_t i=0; i < 7; i++)
  {
    if (str[i] == '-')
    {
      buffer[index] = _tm1637_minus;
      index++;
    }
    else if((str[i] >= '0') && (str[i] <= '9'))
    {
      buffer[index] = _tm1637_digit[str[i] - 48];
      index++;
    }
    else if (str[i] == '.')
    {
      if (index > 0)
        buffer[index - 1] |= _tm1637_dot;      
    }
    else
    {
      buffer[index] = 0;
      break;
    }
  }
  tm1637_write_raw(tm1637, buffer, 6, pos);              
  tm1637_unlock(tm1637);  
}
#endif

//#######################################################################################################################
void tm1637_show_zero(tm1637_t *tm1637, bool enable)
{
  tm1637->show_zero = enable;
}
//#######################################################################################################################
void tm1637_fill(tm1637_t *tm1637, bool enable)
{
	if (enable)
		tm1637_write_segment(tm1637, _tm1637_on, 6, 0);
	else
		tm1637_write_segment(tm1637, _tm1637_off, 6, 0);		
}
//#######################################################################################################################

//#######################################################################################################################








