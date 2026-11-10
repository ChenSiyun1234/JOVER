/*
 * TestDefinition.h
 *
 *  Created on: Oct 26, 2026
 *      Author: Kalani
 */

#pragma once

typedef enum {PASS, FAIL, NEEDS_VERIFICATION} TestStatus;

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
//                           TEST DECLARATIONS
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-


TestStatus testFunction_Nothing();

// extern TestStatus yourTestNameHere();


//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
//                           TEST REGISTRATION
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-

// Function that returns a pointer to a void function
TestStatus (*getTestFunction(int test_id))();

void runAllTests(TestStatus results[]);





