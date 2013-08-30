// Bring in my package's API, which is what I'm testing
#include <se306Project/sheepdog.h>
// Bring in gtest
#include <gtest/gtest.h>

// Test the sheepdog contructor
TEST(sheepdogTest, testConstructor10)
{
	sheepdogNode sheepdog = sheepdogNode(10);
	EXPECT_EQ(10, sheepdog.sheepNum);
}

TEST(sheepdogTest, testConstructor1)
{
	sheepdogNode sheepdog = sheepdogNode(1);
	EXPECT_EQ(10, sheepdog.sheepNum);
}

TEST(sheepdogTest, testConstructor0)
{
	sheepdogNode sheepdog = sheepdogNode(0);
	EXPECT_EQ(10, sheepdog.sheepNum);
}

TEST(sheepdogTest, testConstructorNegOne)
{
	sheepdogNode sheepdog = sheepdogNode(-1);
	EXPECT_EQ(10, sheepdog.sheepNum);
}

TEST(sheepdogTest, testConstructor10Million)
{
	sheepdogNode sheepdog = sheepdogNode(10000000);
	EXPECT_EQ(10, sheepdog.sheepNum);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
