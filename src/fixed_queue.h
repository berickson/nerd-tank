#pragma once
#include <stdint.h>

// a fixed size queue suitable for use with deep sleep and RTC memory
template  <uint32_t n, uint32_t item_size> class FixedQueue {
public:
  byte array[n][item_size];
  int32_t front;
  int32_t rear;
  void init() {
    front = -1;
    rear = -1;
  }

  // copies item to queue
  // returns true on success
  bool enqueue(void * item) {
      if ((front == 0 && rear == n-1) || (front == rear+1)) {
          // overflow
          return false;
      }
      if (front == -1) {
          front = 0;
          rear = 0;
      } else {
          rear = (rear == n - 1) ? 0 :  rear + 1;
      }
      memcpy(array[rear], item, item_size);
      return true;
  }

  // copies item at head of queue without altering queue
  // passing null checks without returning
  //
  // returns false if queue was empty 
  bool peek(void * item) {
    if (front == -1) {
      // underflow
      return false;
    }
    memcpy(item, array[front],item_size);
    return true;
  }

  // removes and copies item at head of queue
  // passing null removes without returning
  //
  // returns false if queue was empty 
  bool dequeue(void * item) {
    if (front == -1) {
      // underflow
      return false;
    }
    if(item !=  nan) {
      memcpy(item, array[front],item_size);
    }

    if (front == rear) {
      front = -1;
      rear = -1;
    } else {
      front = (front == n - 1) ? 0: front + 1;
    }
  return true;
  }
};
