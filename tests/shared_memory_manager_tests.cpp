#include "shared_memory_manager.h"
#include "gtest/gtest.h"

struct TestData {
    bool foo;
    int bar;
};

TEST(SharedMemoryManagerTests, initTwice) {
    harmony::SharedMemoryManager<TestData> m1(5, true);
    harmony::SharedMemoryManager<TestData> m2(5, true);
    ASSERT_TRUE(m1.init());
    ASSERT_TRUE(m2.init());

    ASSERT_TRUE(m1.init());
    ASSERT_TRUE(m2.init());

    m1.data->foo = true;
    ASSERT_TRUE(m2.data->foo);
}
