# Creating and Running Tests

## Test Format
Tests must be written as a global function with no arguments, which returns type TestStatus. This is an enum defined in TestDefinition.h (so please include this file in your test file) with the options PASS, FAIL, and NEEDS_VERIFICATION. They should also include comments specifying how the test should run and what each part does. 

## Including a Test in the System
To include a test in the system, add the following to the document `TestDefinition.h`:
- write your test name with the format `extern TestStatus yourTestNameHere();` under test declarations
Also add the following to `TestDefinition.c`:
- add your test function to the bottom of the array *allTests*
- add a case in *getTestFunction* for your test
Make sure that your test has the same location in *allTests* and *getTestFunction", or running all of the tests will not work properly. Also note that tests must actually return a value, and cannot run indefinitely.

## Running a Specific Test
In order to run a specific test, change the value of RUN_TEST in *main.c* to 1 and change the value of TEST_ID to the case number of the test function in *getTestFunction()*. To run all tests, change the value of RUN_TEST to 2. If you would like to run main proper instead of any tests, make sure the value of RUN_TEST is 0;