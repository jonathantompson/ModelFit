//
//  callback_queue_item.h
//
//  Created by Jonathan Tompson on 7/10/12.
//

#pragma once

namespace jtil {
namespace threading {
  
  // This is simple singly-linked-list item.  Used as a sub-class of queue.
  template <class T>
  struct CallbackQueueItem {
    explicit CallbackQueueItem(const T& value) {
      next = NULL;
      data = value;
    }
    CallbackQueueItem* next;
    T data;
  };
  
};  // namespace threading
};  // namespace jtil
