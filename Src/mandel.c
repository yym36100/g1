#include <stdint.h>

#define limit 250.0

extern uint8_t *dpo;
extern uint16_t pi;

#define width 240.0
#define height 320.0

void mysetpixel(uint16_t x, uint16_t y,uint32_t color);

inline uint16_t checkpoint(float cx, float cy)
{
	uint8_t res = 0;
	uint16_t i = 0;
	float x=cx,y=cy,xx,yy,xy;
	do{
		xx= x*x;
		yy = y*y;
		xy = x*y;
		x = xx-yy+cx;
		y = 2*xy+cy;
		i++;
	}while((xx+yy)<4.0 && i<limit);
	return i;
} 

void drawset2(float sx,float ex, float sy, float ey)
{
	uint32_t c;uint8_t cc;
	float cx,cy;
	uint8_t val;
	float dx,dy;

	dx = (ex-sx)/width;
	dy = (ey-sy)/height;

	cy = sy;
	for(uint16_t y=0;y<height;y++)
	{	
		cy+=dy;
		cx = sx;		
		for(uint16_t x=0;x<width;x++)
		{						
			cx+=dx;
			val = checkpoint(cx,cy);			
			cc = val;
			//cc = 0xff;

			c = 0xff + (cc<<8)+(cc<<16)+(cc<<24);
			mysetpixel(x,y,c);
		}
	}
}

void mysetpixel(uint16_t x, uint16_t y,uint32_t color){
	uint8_t *dp = dpo + y*pi + x*4;
	*dp++ = (color>>24)&0xff;
	*dp++ = (color>>16)&0xff;
	*dp++ = (color>>8)&0xff;				
	*dp++ = color&0xff;
}
