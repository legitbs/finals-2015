//
//  my_functions.h
//  robotbattle
//
//  Created by Lightning on 1/21/15.
//  Copyright (c) 2015 Lightning. All rights reserved.
//

#ifndef robotbattle_my_functions_h
#define robotbattle_my_functions_h


typedef struct MyDouble
{
	union
	{
		unsigned int IntSize;
		struct
		{
			unsigned char Signed : 1;
			unsigned short SubNumber : 14;
			unsigned int Fraction : 17;
		};
	};
} MyDouble;

int my_snprintf(char *Buf, int MaxBufSize, const char *fmt, ...);
int my_printf(const char *fmt, ...);
int my_strcmp(char *a, char *b);
void LowerCase(char *Line);
MyDouble my_atof(char *a);
int GetSystemVarID(char *name);
int my_atoi(char *a);
int my_strlen(char *a);
int my_isnumeric(char *a);
int SplitData(char *Line, char *Entries[], char SplitChar, int StripLine, int MaxCount);
int my_numlen(int val);

double MyDoubleToDouble(MyDouble A);
MyDouble DoubleToMyDouble(double A);
int MyDouble_EQUAL(MyDouble A, MyDouble B);
MyDouble MyDouble_ADD(MyDouble A, MyDouble B);
MyDouble MyDouble_SUB(MyDouble A, MyDouble B);
MyDouble MyDouble_MUL(MyDouble A, MyDouble B);
MyDouble MyDouble_DIV(MyDouble A, MyDouble B);
MyDouble MyDouble_MOD(MyDouble A, MyDouble B);
MyDouble MyDouble_POW(MyDouble A, MyDouble B);
MyDouble MyDouble_NEG(MyDouble A);

#undef printf
#undef snprintf
#undef strcmp
#undef atoi
#undef strlen
#undef isnumeric

#define MyDouble_EQUAL(A, B) \
( \
	((A.Signed == B.Signed) && (A.SubNumber == B.SubNumber) && (A.Fraction == B.Fraction)) \
) \

#define MyDouble_LESSTHAN(A, B) \
( \
	(A.Signed && !B.Signed) || \
	((A.Signed == B.Signed) && (A.Signed == 0) && ((A.SubNumber < B.SubNumber) || ((A.SubNumber == B.SubNumber) && (A.Fraction < B.Fraction)))) || \
	((A.Signed == B.Signed) && (A.Signed == 1) && ((A.SubNumber > B.SubNumber) || ((A.SubNumber == B.SubNumber) && (A.Fraction > B.Fraction)))) \
)

#define MyDouble_GREATERTHAN(A, B) \
( \
	(!A.Signed && B.Signed) || \
	((A.Signed == B.Signed) && (A.Signed == 0) && ((A.SubNumber > B.SubNumber) || ((A.SubNumber == B.SubNumber) && (A.Fraction > B.Fraction)))) || \
	((A.Signed == B.Signed) && (A.Signed == 1) && ((A.SubNumber < B.SubNumber) || ((A.SubNumber == B.SubNumber) && (A.Fraction < B.Fraction)))) \
)

#define MyDouble_LESSTHAN_EQUAL(A, B) \
( \
	MyDouble_EQUAL(A, B) || MyDouble_LESSTHAN(A, B) \
)

#define MyDouble_GREATERTHAN_EQUAL(A, B) \
( \
	MyDouble_EQUAL(A, B) || MyDouble_GREATERTHAN(A, B) \
)

#define MyDouble_ABS(A) \
( \
	(MyDouble){{.Signed = 0, .SubNumber = A.SubNumber, .Fraction = A.Fraction}} \
)

#define ImmToMyDouble(A) ((MyDouble){{.Signed = (A < 0), .SubNumber = (unsigned short)(abs(A)), .Fraction = 0}})
#define FloatToMyDouble(A) ((MyDouble){{.Signed = (A < 0), .SubNumber = (unsigned short)(A), .Fraction = (((int)((float)(A) * 100000.0 + 0.000001) % 100000))}})
#define MyDouble0 ((MyDouble){{.Signed = 0, .SubNumber = 0, .Fraction = 0}})

#endif
