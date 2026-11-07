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

void runAllTests(TestStatus (results[sizeof(all_tests) / sizeof(all_tests[0])])){
	int size = sizeof(all_tests) / sizeof(all_tests[0]);
	for(int i = 0; i < size; ++i){
		TestStatus (*testFunc)();
		testFunc = getTestFunction(i);
		results[i] = testFunc();
	}
}





