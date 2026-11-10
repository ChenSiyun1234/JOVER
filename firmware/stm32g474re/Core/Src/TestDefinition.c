
#include "TestDefinition.h"

//default test
TestStatus testFunction_Nothing() {
	TestStatus status = PASS;
	return status;
};

// array with pointers to all tests
TestStatus (*all_tests[])() = {
		testFunction_Nothing,
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
