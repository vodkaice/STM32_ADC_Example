/*my string function*/
#include "str.h"
int strcmp(const char *str1, const char *str2)
{
    while(1){
	if(*str1 == '\0' || *str2 == '\0'){
	    if(*str1 == *str2) return 1;
	    return 0;
	}
	else if ( *str1++ != *str2++ ) break;
    }
    return 0;
}

int strlen(const char *str)
{
    int length = 0;
    const char *s = str;
    while(*s++)
	length++;
    return length;
}

void reverse(char *str)
{
    int i,length = strlen(str);
    char temp;
    for(i = 0; i<length; i++, length-- ){
	temp = str[i];
	str[i] = str[length-1];
	str[length-1] = temp;
    }
}

void itoa(int n, char *str)
{
    int i,sign;
    if((sign = n) < 0)
	n = -n;
    i = 0;
    do {
	str[i++] = n % 10 + '0';
    } while((n /= 10) > 0);
    if(sign < 0)
	str[i++] = '-';
    str[i] = '\0';
    reverse(str);
}
