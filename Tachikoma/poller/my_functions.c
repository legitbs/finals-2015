#include <stdarg.h>
#include <stdio.h>
#include <fcntl.h>
#include <memory.h>
#include <unistd.h>
#include <math.h>
#include "robotbattle.h"

double MyDoubleToDouble(MyDouble A)
{
	double B;
	B = (double)A.SubNumber;
	B += ((double)A.Fraction * 0.00001);
	if(A.Signed && B != 0)
		B = -B;
	return B;
}

MyDouble DoubleToMyDouble(double A)
{
	MyDouble B;
        double C;

	if((A > -16384.00) && (A < 16384.0))
	{
		B.SubNumber = fabs(A);
		B.Signed = (A < 0);
	}
	else
	{
		A = fabs(A) - 16384.0;
		if(A > 16383.0)
			B.SubNumber = 16383;
		else
			B.SubNumber = A;
		B.Signed = 1;
	}

	//get the fractional part
	C = fmod(fabs(A), 1.0);
	C *= 100000;

	B.Fraction = (unsigned int)(C);
	return B;
}

MyDouble MyDouble_NEG(MyDouble A)
{
	if((A.SubNumber == 0) && (A.Fraction == 0))
		return A;

	A.Signed = !A.Signed;
	return A;
}

MyDouble MyDouble_ADD(MyDouble A, MyDouble B)
{
	double dA, dB;
	dA = MyDoubleToDouble(A);
	dB = MyDoubleToDouble(B);
	return DoubleToMyDouble(dA + dB);
}

MyDouble MyDouble_SUB(MyDouble A, MyDouble B)
{
	double dA, dB;
	dA = MyDoubleToDouble(A);
	dB = MyDoubleToDouble(B);
	return DoubleToMyDouble(dA - dB);
}

MyDouble MyDouble_MUL(MyDouble A, MyDouble B)
{
	double dA, dB;
	dA = MyDoubleToDouble(A);
	dB = MyDoubleToDouble(B);
	return DoubleToMyDouble(dA * dB);
}

MyDouble MyDouble_DIV(MyDouble A, MyDouble B)
{
	double dA, dB;
	dA = MyDoubleToDouble(A) + 0.000001;
	dB = MyDoubleToDouble(B);
	return DoubleToMyDouble(dA / dB);
}

MyDouble MyDouble_MOD(MyDouble A, MyDouble B)
{
	double dA, dB;
	dA = MyDoubleToDouble(A) + 0.000001;
	dB = MyDoubleToDouble(B);
	return DoubleToMyDouble(fmod(dA, dB));
}

MyDouble MyDouble_POW(MyDouble A, MyDouble B)
{
	double dA, dB;
	dA = MyDoubleToDouble(A);
	dB = MyDoubleToDouble(B);
	return DoubleToMyDouble(powf(dA, dB));
}

int my_snprintf_internal(char *Buf, int MaxBufSize, const char *fmt, va_list *list)
{
    int DataLen = 0;
    
    while(*fmt)
    {
        //if out of room then stop printing
        if(DataLen >= (MaxBufSize - 1))
            break;
        
        if(*fmt != '%')
        {	
            Buf[DataLen] = *fmt;
            DataLen++;
        }
        else
        {
            //% marker, check next char
            switch(*(fmt+1))
            {
                case '%':
                    Buf[DataLen] = '%';
                    DataLen++;
                    fmt++;
                    break;
                    
                case 's':
                {
                    char *str = va_arg(*list, char *);
                    int len = strlen(str);
                    if((len + DataLen + 1) >= MaxBufSize)
                        len = MaxBufSize - DataLen - 1;
                    
                    memcpy(&Buf[DataLen], str, len);
                    DataLen += len;
                    
                    fmt++;
                    break;
                }
                    
                case 'd':
                {
                    char numbuf[15];
                    int Pos = sizeof(numbuf) - 1;
                    int numlen = 0;
                    int origval = va_arg(*list, int);
                    int val = __builtin_abs(origval);
                    memset(numbuf, 0, sizeof(numbuf));
                    if(val == 0)
                    {
                        numbuf[Pos] = 0x30;
                        Pos--;
                        numlen++;
                    }
                    else
                    {
                        while(val)
                        {
                            numbuf[Pos] = 0x30 + (val % 10);
                            val = (val - (val % 10)) / 10;
                            Pos--;
                            numlen++;
                        };
                    }
                    
                    if(origval < 0)
                    {
                        numbuf[Pos] = '-';
                        Pos--;
                        numlen++;
                    }
                    
                    Pos++;
                    if(numlen + DataLen + 1 >= MaxBufSize)
                        numlen = MaxBufSize - DataLen - 1;
                    memcpy(&Buf[DataLen], &numbuf[Pos], numlen);
                    DataLen += numlen;
                    fmt++;
                    break;
                }

                case 'f':
                {
                    char numbuf[15];
                    int Pos = sizeof(numbuf) - 1;
                    int numlen = 0;
                    union
                    {
                        MyDouble d;
                        unsigned int i;
                    } origval;
                    origval.i = va_arg(*list, unsigned int);
                    int val = origval.d.SubNumber;
                    memset(numbuf, 0, sizeof(numbuf));
                    if(val == 0)
                    {
                        numbuf[Pos] = 0x30;
                        Pos--;
                        numlen++;
                    }
                    else
                    {
                        while(val)
                        {
                            numbuf[Pos] = 0x30 + (val % 10);
                            val = (val - (val % 10)) / 10;
                            Pos--;
                            numlen++;
                        };
                    }
                    
                    if(origval.d.Signed)
                    {
                        numbuf[Pos] = '-';
                        Pos--;
                        numlen++;
                    }
                    
                    Pos++;
                    if(numlen + DataLen + 1 >= MaxBufSize)
                        numlen = MaxBufSize - DataLen - 1;
                    memcpy(&Buf[DataLen], &numbuf[Pos], numlen);
                    DataLen += numlen;

                    //insert the period
                    Buf[DataLen] = '.';
                    DataLen++;

                    //get the fractional part
                    Pos = sizeof(numbuf) - 1;
                    numlen = 0;
                    val = origval.d.Fraction;
                    memset(numbuf, 0x30, sizeof(numbuf));
                    while(val)
                    {
                        numbuf[Pos] = 0x30 + (val % 10);
                        val = (val - (val % 10)) / 10;
                        Pos--;
                        numlen++;
                    };

                    //fractional part is always 5 digits long
                    Pos = sizeof(numbuf) - 5;
                    numlen = 5;
                    if(numlen + DataLen + 1 >= MaxBufSize)
                        numlen = MaxBufSize - DataLen - 1;
                    memcpy(&Buf[DataLen], &numbuf[Pos], numlen);
                    DataLen += numlen;
                    fmt++;
                    break;
                }

                case 'F':
                {
                    //floating point value
                    char numbuf[30];
                    char numbuf2[10];
                    int Pos = 0;
                    int CurPos = sizeof(numbuf) - 1;
                    double val = va_arg(*list, double);
                    unsigned long long intval;

                    memset(numbuf, 0, sizeof(numbuf));
                    memset(numbuf2, 0, sizeof(numbuf2));

                   
                    //get the integer version
                    intval = (long long)val;
                    if(val < 0.0f)
                    {
                        //assign negative sign
                        numbuf[Pos] = '-';
                        Pos++;

                        val = fabs(val - (double)intval);
                        intval = -intval;
                    }
                    else
                        val -= (double)intval;

                    //start parsing it up
                    if(intval == 0x8000000000000000)
                    {
                        //val is too large to print in this manner, just put in a few x's to indicate an unknown
                        numbuf[CurPos] = 'x';
                        numbuf[CurPos-1] = 'x';
                        numbuf[CurPos-2] = 'x';
                        CurPos -= 3;
                    }
                    else if(intval == 0)
                    {
                        numbuf[CurPos] = '0';
                        CurPos--;
                    }
                    else
                    {
                        while(intval)
                        {
                            numbuf[CurPos] = 0x30 + (intval % 10);
                            CurPos--;
                            intval /= 10;
                        };
                    }

                    //shift all the numbers over now
                    memcpy(&numbuf[Pos], &numbuf[CurPos+1], sizeof(numbuf) - CurPos - 1);
                    Pos += sizeof(numbuf) - CurPos - 1;
                    //we now have the first part, add the dot
                    numbuf[Pos] = '.';
                    Pos++;
                    
                    //now figure out the fractional part to the best of our ability
                    intval = (val * 100000000.0f);
                    //intval = (unsigned int)(val * 100.0f);
                    if(intval == 0)
                    {
                        numbuf[Pos] = '0';
                        Pos++;
                    }
                    else
                    {
                        CurPos = sizeof(numbuf2) - 1;
                        while(intval)
                        {
                            numbuf2[CurPos] = 0x30 + (intval % 10);
                            CurPos--;
                            intval /= 10;
                        };
                        memcpy(&numbuf[Pos], &numbuf2[CurPos+1], sizeof(numbuf2) - CurPos - 1);
                        Pos += sizeof(numbuf2) - CurPos - 1;
                    }
                    
                    //print it out
                    if(Pos + DataLen + 1 >= MaxBufSize)
                        Pos = MaxBufSize - DataLen - 1;
                    memcpy(&Buf[DataLen], numbuf, Pos);
                    DataLen += Pos;
                    fmt++;

                    break;
                }
                    
                default:
                    Buf[DataLen] = *fmt;
                    DataLen++;
            };
        }
        
        fmt++;
    };

    //make sure we can null terminate at the end of the buffer
    if(DataLen >= (MaxBufSize - 1))
        DataLen = MaxBufSize - 1;
    Buf[DataLen] = 0;
    
    return DataLen;
}

int my_numlen(int val)
{
    int Pos = 0;
    
    if(val == 0)
        return 1;
    
    //figure out how many digits this value is
    while(val)
    {
        val = (val - (val % 10)) / 10;
        Pos++;
    };
    
    return Pos;
}

int my_printf(const char *fmt, ...)
{
    char    Buf[512];
    int     ret;
    va_list list;
    
    va_start(list, fmt);
    ret = my_snprintf_internal(Buf, sizeof(Buf), fmt, &list);
    write(1, Buf, ret);
    va_end(list);
    return ret;
}

int my_snprintf(char *Buf, int MaxBufSize, const char *fmt, ...)
{
    int     ret;
    va_list list;
    
    va_start(list, fmt);
    ret = my_snprintf_internal(Buf, MaxBufSize, fmt, &list);
    va_end(list);
    return ret;
}

int my_strcmp(char *a, char *b)
{
    while(1)
    {
        if(*a == *b)
        {
            if(*a == 0)
                return 0;
            a++;
            b++;
        }
        else
            return 1;
    };
    
    return 1;
}

void LowerCase(char *Line)
{
    int i;
    for(i = 0; Line[i] != 0; i++)
    {
        if(Line[i] >= 'A' && Line[i] <= 'Z')
            Line[i] |= 0x20;
    }
}

MyDouble my_atof(char *a)
{
    MyDouble Val = MyDouble0;
    int neg = 0;
    int temp;
    int fraccount;

    if(*a == '-')
    {
        neg = 1;
        a++;
    }

    //get top part
    while((*a != '.') && (*a != 0))
    {
        temp = *a - 0x30;
        if ((temp < 0) || (temp > 9))
        {
            ErrorMsg = "Invalid value";
            NEXT_STATE(PARSE_ERROR);
        }
        
        Val.SubNumber *= 10;
        Val.SubNumber += temp;
        a++;
    }
    
    if(*a == '.')
    {
        a++;

        fraccount = 0;
        while(*a != 0)
        {
            temp = *a - 0x30;
            if ((temp < 0) || (temp > 9))
            {
                ErrorMsg = "Invalid value";
                NEXT_STATE(PARSE_ERROR);
            }
            
            Val.Fraction *= 10;
            Val.Fraction += temp;
            a++;
            fraccount++;
        }

	while(fraccount < 5)
        {
            Val.Fraction *= 10;
            fraccount++;
        }
    }

    if(neg)
        Val.Signed = 1;
    
    return Val;
}

int my_atoi(char *a)
{
    unsigned int i = 0;
    unsigned int Neg = 1;
    unsigned int DotSeen = 0;

    if(*a == '-')
    {
        Neg = -1;
        a++;
    }

    while(*a)
    {
        if((*a < 0x30) || (*a > 0x39))
        {
            if((*a == '.') && !DotSeen)
                DotSeen = 1;
            else
                break;
        }

        //need integer only so don't add fractional parts
        if(!DotSeen)
        {
            i *= 10;
            i += (*a - 0x30);
        }

        a++;
    }
    
    return i * Neg;
}

int my_isnumeric(char *a)
{
    int DotSeen = 0;

    //if first character is a negative then allow it
    if(*a == '-')
        a++;

    while(*a)
    {
        if((*a < 0x30) || (*a > 0x39))
        {
            if((*a == '.') && !DotSeen)
                DotSeen = 1;
            else
                return 0;
        }
        a++;
    }
    
    return 1;
}

int GetSystemVarID(char *name)
{
    char *Names[] = {
        "_true",
        "_false",
        "_result",
        "_name",
        "_gunheat",
        "_energy",
        "_bodyaim",
        "_radaraim",
        "_gunaim",
        "_accel",
        "_velocity",
        "_speedtarget",
        "_rotating",
        "_moving",
        "_xpos",
        "_ypos",
        "_movermn",
        "_bodyrmn",
        "_radarrmn",
        "_gunrmn",
        "_cldbearing",
        "_cldheading",
        "_dtcheading",
        "_dtcbearing",
        "_dtcenergy",
        "_dtcdistance",
        "_ping",
        "_deceltarget",
        "_accelltarget",
        "_internal_result",
    };
    
    //ct_assert((sizeof(Names) / sizeof(char*)) == SYSTEM_VAR_COUNT);
           
    int i;
    for(i = 0; i < SYSTEM_VAR_COUNT; i++)
    {
        if(my_strcmp(Names[i], name) == 0)
            return i;
    }
    
    return -1;
}

int my_strlen(char *a)
{
    int i = 0;
    for(i = 0; a[i] != 0; i++){}
    return i;
}

void StripData(char *Line)
{
    //find the end of the line then walk backwards removing all spaces
    int i;
    int LastPos = -1;
    
    //walk forwards looking for null while noting the last non-space character
    for(i = 0; Line[i] != 0x00; i++)
    {
        if(Line[i] != 0x20)
            LastPos = i;
    }
    
    //set the next character after us as null, if the line is null or
    //the first character and all future characters are spaces then -1+1 = 0 for first char
    Line[LastPos+1] = 0;
    return;
}

int SplitData(char *Line, char *Entries[], char SplitChar, int StripLine, int MaxCount)
{
    //parse up a line and return it in blocks
    int i;
    int NumEntries;
    
    NumEntries = 1;
    
    if(Entries != 0)
        Entries[0] = &Line[0];
    
    if(StripLine)
        StripData(Line);
    
    for(i = 0; Line[i] != 0; i++)
    {
        if(Line[i] == SplitChar)
        {
            if(Entries != 0)
            {
                Line[i] = 0x00;
                Entries[NumEntries] = &Line[i]+1;
            }
            NumEntries++;
            if((MaxCount > 0) && (NumEntries == MaxCount))
                break;
        }
    }

    //if not enough splits then null out the empty ones
    if((MaxCount != -1) && (NumEntries != MaxCount))
        memset(&Entries[NumEntries], 0, sizeof(char *)*(MaxCount-NumEntries));
    return NumEntries;
}
