
#include "TestDefinition.h"

//default test
TestStatus testFunction_Nothing() {
	TestStatus status = PASS;
	return status;
};

// array with pointers to all tests
TestStatus (*all_tests[])() = {
		testFunction_Nothing,
		testFunction_UART_TXRX_Loopback,
		testFunction_UART_TX_External,
};

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
//                           TEST REGISTRATION
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-

// Function that returns a pointer to a void function
TestStatus (*getTestFunction(int test_id))() {
	//! Every case must return an enum of TestStatus
	switch(test_id){
		case 0:
			return testFunction_Nothing;
		case 1:
			return testFunction_UART_TXRX_Loopback;
		case 2:
			return testFunction_UART_TX_External;
	}
	return testFunction_Nothing;
}

void runAllTests(TestStatus results[]){
	int size = sizeof(all_tests) / sizeof(all_tests[0]);
	for(int i = 0; i < size; ++i){
		TestStatus (*testFunc)();
		testFunc = getTestFunction(i);
		results[i] = testFunc();
	}
}
