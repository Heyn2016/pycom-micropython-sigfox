#ifndef MODM100_H
#define	MODM100_H

unsigned int setRFPower( unsigned char  power, unsigned char *pbuf );
unsigned int getRFPower(                       unsigned char *pbuf );
unsigned int query     ( unsigned short loop , unsigned char *pbuf );

unsigned int getQueryParam  ( unsigned char *pbuf );
unsigned int setQueryParam  ( unsigned char select,
                              unsigned char session,
                              unsigned char target,
                              unsigned char qvalue,
                              unsigned char *pbuf );
unsigned int stop      (                       unsigned char *pbuf );

#endif /* MODM100_H */
