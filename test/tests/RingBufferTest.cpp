#include "gtest/gtest.h"

#include "Arduino.h"
#include "../fake/Logger.h"

#include "RingBuffer.h"
// #include "../src/RingBuffer.cpp"


TEST(RingBufferTest, canNotPutMoreBytesThanSize)
{
    RingBuffer Rb(2);
    char bytes[3] = { '1', '2', '3' };

    EXPECT_EQ(Rb.putBytes(bytes, 3), RingBuffer::STATUS_FULL);
}

TEST(RingBufferTest, canPutBytesInBuffer)
{
    RingBuffer Rb(10);
    char bytes[3] = { '1', '2', '3' };

    EXPECT_EQ(Rb.putBytes(bytes, 3), RingBuffer::STATUS_OK);
}

TEST(RingBufferTest, getMessageFromBuffer)
{
    RingBuffer Rb(10);
    char bytes[3] = { '1', '2', '3' };

    Rb.putBytes(bytes, 3);


    char returnedMessage[10];
    int  length = Rb.getMessage(returnedMessage);

    EXPECT_EQ(length,             3);
    EXPECT_EQ(returnedMessage[1], bytes[1]);
}

TEST(RingBufferTest, addENDAfterMessage)
{
    RingBuffer Rb(10);
    char bytes[3] = { '1', '2', '3' };

    Rb.putBytes(bytes, 3);

    EXPECT_EQ(Rb.get(), '1');
    EXPECT_EQ(Rb.get(), '2');
    EXPECT_EQ(Rb.get(), '3');
    EXPECT_EQ(Rb.get(), RingBuffer::END);
}

TEST(RingBufferTest, getFromBack)
{
    RingBuffer Rb(10);
    char bytes[3] = { '1', '2', '3' };

    Rb.putBytes(bytes, 3);

    EXPECT_EQ(Rb.getFromBack(), RingBuffer::END);
    EXPECT_EQ(Rb.getFromBack(), '3');
    EXPECT_EQ(Rb.getFromBack(), '2');
    EXPECT_EQ(Rb.getFromBack(), '1');
}
