/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef MESSAGE_FILTERS_SIGNAL_H
#define MESSAGE_FILTERS_SIGNAL_H

#include <boost/noncopyable.hpp>

#include "connection.h"
#include "null_types.h"
#include <ros/message_event.h>
#include <ros/parameter_adapter.h>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace message_filters
{
using ros::ParameterAdapter;

template<typename ...M>
class CallbackHelper9
{
public:

  virtual ~CallbackHelper9() {}

  virtual void call(bool nonconst_force_copy, const ros::MessageEvent<M const>& ...) = 0;

  typedef boost::shared_ptr<CallbackHelper9> Ptr;
};

template<typename ...P>
class CallbackHelper9T :public CallbackHelper9<typename ParameterAdapter<P>::Message ...>
{
public:

  CallbackHelper9T(const boost::function<void(typename ParameterAdapter<P>::Parameter ...)>& cb)
  : callback_(cb)
  {
  }

  virtual void call(bool nonconst_force_copy, const typename ParameterAdapter<P>::Event& ...args)
  {
	  callback_(typename ParameterAdapter<P>::getParameter(args) ...);
  }

private:
  boost::function<void(typename ParameterAdapter<P>::Parameter ...)> callback_;
};

template<typename ...M>
class Signal9
{
  //typedef boost::shared_ptr<CallbackHelper9<typename M ...> > CallbackHelper9Ptr;
  //typedef std::vector<CallbackHelper9Ptr> V_CallbackHelper9;

public:
  //typedef const boost::shared_ptr<NullType const>& NullP;

  template<typename ...P>
  Connection addCallback(const boost::function<void(P ...)>& callback)
  {
    CallbackHelper9T<P ...>* helper = new CallbackHelper9T<P ...>(callback);

    boost::mutex::scoped_lock lock(mutex_);
    callbacks_.push_back(CallbackHelper9Ptr(helper));
    return Connection(boost::bind(&Signal9::removeCallback, this, callbacks_.back()));
  }

  void removeCallback(const boost::shared_ptr<CallbackHelper9<M ...> >& helper)
  {
    boost::mutex::scoped_lock lock(mutex_);
    typename std::vector<boost::shared_ptr<CallbackHelper9<M ...> > >::iterator it = std::find(callbacks_.begin(), callbacks_.end(), helper);
    if (it != callbacks_.end())
    {
      callbacks_.erase(it);
    }
  }

  void call(const ros::MessageEvent<M const>& ...args)
  {
    boost::mutex::scoped_lock lock(mutex_);
    bool nonconst_force_copy = callbacks_.size() > 1;
    typename std::vector<boost::shared_ptr<CallbackHelper9<M ...> > >::iterator it = callbacks_.begin();
    typename std::vector<boost::shared_ptr<CallbackHelper9<M ...> > >::iterator end = callbacks_.end();
    for (; it != end; ++it)
    {
      const boost::shared_ptr<CallbackHelper9<M ...> >& helper = *it;
      helper->call(nonconst_force_copy, args...);
    }
  }

private:
  boost::mutex mutex_;
  std::vector<boost::shared_ptr<CallbackHelper9<M ...> > > callbacks_;
};

} // message_filters

#endif // MESSAGE_FILTERS_SIGNAL_H


