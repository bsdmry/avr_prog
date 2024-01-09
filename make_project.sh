mkdir $1
echo "MCU = atmega8" > $1/Makefile
echo "CPU_SPEED = 8000000UL" >> $1/Makefile
echo "TARGET = "$1 >> $1/Makefile
echo "include ../Makefile.include" >> $1/Makefile
touch $1/$1.c
touch $1/$1.h

echo "#define F_CPU 8000000UL" >> $1/$1.c
echo "#include <avr/io.h>" >> $1/$1.c
echo "#include <util/delay.h>" >> $1/$1.c
echo "#include \""$1".h\"" >> $1/$1.c
echo "" >> $1/$1.c
echo "int main(void)" >> $1/$1.c
echo "{" >> $1/$1.c
echo "while(1){};" >> $1/$1.c
echo "}" >> $1/$1.c
