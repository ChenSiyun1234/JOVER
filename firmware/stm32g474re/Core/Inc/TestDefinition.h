/*
 * TestDefinition.h
 *
 *  Created on: Oct 26, 2026
 *      Author: Kalani
 */



typedef enum {PASS, FAIL, NEEDS_VERIFICATION} TestStatus;

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
//                           TEST DECLARATIONS
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-


TestStatus testFunction_Nothing() {
	TestStatus status = PASS;
	return status;
};

// extern TestStatus yourTestNameHere();

extern TestStatus testFunction_UART_TXRX_Loopback();
extern TestStatus testFunction_UART_TX_External();
extern TestStatus testFunction_UART_RX_External();



// array with pointers to all tests
TestStatus (*all_tests[])() = {
		testFunction_Nothing,
		testFunction_UART_TXRX_Loopback,
		testFunction_UART_TX_External,
		testFunction_UART_RX_External
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
		case 3:
			return testFunction_UART_RX_External;
	}
	return testFunction_Nothing;
}

void runAllTests(TestStatus (results[sizeof(all_tests) / sizeof(all_tests[0])])){
	int size = sizeof(all_tests) / sizeof(all_tests[0]);
	for(int i = 0; i < size; ++i){
		TestStatus (*testFunc)();
		testFunc = getTestFunction(i);
		results[i] = testFunc();
	}
}





