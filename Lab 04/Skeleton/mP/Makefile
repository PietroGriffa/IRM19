all:  serialLib serialTest 
	

serialLib:
	gcc -c udoo_serial.c RMS.c digital_filter.c 


serialTest: serialLib
	gcc -o lab04_test udoo_serial.o digital_filter.o RMS.o lab04_test.c -lm
	
clean:
	rm -rf udoo_serial.o digital_filter.o RMS.o lab04_test.o lab04_test
