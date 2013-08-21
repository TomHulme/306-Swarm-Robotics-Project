// Bring in my package's API, which is what I'm testing
#include <se306Project/Field.h>
// Bring in gtest
#include <gtest/gtest.h>

//definition of sunData
const int FieldNode::sunData[12] = { 0, 0, 0, 10, 20, 40, 70, 90, 100, 100, 100,
		100 };

// Declare a test
TEST(FieldTest, testRainValue)
{
	FieldNode field = FieldNode(0, (double)5, (double)5);
	field.rain = 10;
	EXPECT_EQ(10, field.rain);
}

TEST(FieldTest, testSoilValue)
{
	FieldNode field = FieldNode(0, (double)5, (double)5);
	EXPECT_EQ((SoilQuality)NORMAL, field.soil);
	field.rain = 1;
	field.setSoil();
	EXPECT_EQ((SoilQuality)ARID, field.soil);
	field.rain = 99;
	field.setSoil();
	EXPECT_EQ((SoilQuality)FERTILE, field.soil);
}

TEST(FieldTest, testSunValue)
{
	FieldNode field = FieldNode(0, (double)5, (double)5);
	EXPECT_EQ(0, field.sunLight);
	field.setSunLight();
	field.setSunLight();
	field.setSunLight();
	field.setSunLight();
	EXPECT_EQ(10, field.sunLight);
	field.setSunLight();
	EXPECT_EQ(20, field.sunLight);
	field.setSunLight();
	EXPECT_EQ(40, field.sunLight);
	field.setSunLight();
	EXPECT_EQ(70, field.sunLight);
	field.setSunLight();
	EXPECT_EQ(90, field.sunLight);
	field.setSunLight();
	EXPECT_EQ(100, field.sunLight);
	field.setSunLight();
	EXPECT_EQ(100, field.sunLight);
	field.setSunLight();
	EXPECT_EQ(100, field.sunLight);
	field.setSunLight();
	EXPECT_EQ(100, field.sunLight);
	field.setSunLight();
	EXPECT_EQ(100, field.sunLight);
	field.setSunLight();
	EXPECT_EQ(100, field.sunLight);
	field.setSunLight();
	EXPECT_EQ(100, field.sunLight);
	field.setSunLight();
	EXPECT_EQ(90, field.sunLight);
}	

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
