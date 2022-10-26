# Test the testing framework is enabled and configured.
add_test(Test_Test arwain_test Test_Test)

# Test the quaternion library.
add_test(Test_QuatenrionConstructors arwain_test Test_QuaternionConstructors)
add_test(Test_QuaternionInverse arwain_test Test_QuaternionInverse)
add_test(Test_QuaternionSum arwain_test Test_QuaternionSum)
add_test(Test_QuaternionAxisAngleConstructor arwain_test Test_QuaternionAxisAngleConstructor)
