/* See page 312 of LPC111x/LPC11Cxx User manual */
#define IAP_LOCATION 0x1fff1ff1

typedef void (*IAP)(unsigned int [], unsigned int []);
IAP iap_entry;
