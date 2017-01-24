/*
   modified: maximilian@glumb.de
   original:

   Arduino Buffered Serial
   A library that helps establish buffered serial communication with a
   host application.
   Copyright (C) 2010 Sigurður Örn Aðalgeirsson (siggi@media.mit.edu)
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.
   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.
   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <Arduino.h>

class RingBuffer
{
public:

  RingBuffer(unsigned int buf_size);

  // This method resets the buffer into an original state (with no data)
  void clear();

  // This releases resources for this buffer, after this has been called the buffer should NOT be used
  void deAllocate();

  // Returns how much space is left in the buffer for more data
  int  getSize();

  // Returns the maximum capacity of the buffer
  int  getCapacity();

  // This method returns the byte that is located at index in the buffer but doesn't modify the buffer like the get methods (doesn't remove
  // the retured byte from the buffer)
  byte peek(unsigned int index);

  //
  // Put methods, either a regular put in back or put in front
  //


  int putBytes(byte        *message,
                 unsigned int messageLength);

  int putBytesInFront(byte        *message,
                        unsigned int messageLength);


  //
  // Get methods, either a regular get from front or from back
  //
  byte get();
  byte getByte();
  byte getFromBack();

  // static const unsigned char START;
  static const unsigned char END;
  static const unsigned char EMPTY;
  static const unsigned int  STATUS_OK;
  static const unsigned int  STATUS_FULL;

private:

  byte *data;

  unsigned int capacity;
  unsigned int position;
  unsigned int length;

  void put(byte in);
  void putInFront(byte in);
};
