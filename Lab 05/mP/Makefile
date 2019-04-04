all:  serialLib serialTest 
	

serialLib:
	gcc -c digital_filter.c lab05_Task1.c


serialTest: serialLib
	gcc -o lab05_test digital_filter.o lab05_Task1.o lab05_test.c -lm
	
clean:
	rm -rf  digital_filter.o lab05_Task1.o lab05_test.o lab05_test
