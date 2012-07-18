/*
 * Copyright 2011 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <gr_uhd_amsg_source.h>
#include <boost/bind.hpp>
#include <gruel/thread.h>
#include <cstdio>

/***********************************************************************
 * UHD Asynchronous Message Source Impl
 **********************************************************************/
class uhd_amsg_source_impl : public uhd_amsg_source{
public:
    uhd_amsg_source_impl(
        const uhd::device_addr_t &device_addr,
        gr_msg_queue_sptr msgq
    ):
        _msgq(msgq), _running(true)
    {
        _dev = uhd::usrp::multi_usrp::make(device_addr);
        _amsg_thread =
        gruel::thread(boost::bind(&uhd_amsg_source_impl::recv_loop, this));
    }

    ~uhd_amsg_source_impl()
    {
        _running = false;
        _amsg_thread.join();
    }

    void recv_loop()
    {
        gr_message_sptr msg;
        uhd::async_metadata_t *md;

        while (_running) {
            msg = gr_make_message(0, 0.0, 0.0, sizeof(uhd::async_metadata_t));
            md = (uhd::async_metadata_t *) msg->msg();

            while (!_dev->get_device()->recv_async_msg(*md, 0.1)) {
                if (!_running)
                    return;
            }

            post(msg);
        }
    }

    void post(gr_message_sptr msg)
    {
        //_msgq->insert_tail(msg);
        printf("POSTED!\n"); fflush(stdout);
        uhd::async_metadata_t *md = (uhd::async_metadata_t *) msg->msg();
        uhd::time_spec_t _time_spec = uhd::time_spec_t(0.0);
        if(md->has_time_spec) {
            _time_spec = md->time_spec;
        }
        uhd::async_metadata_t::event_code_t _event_code = md->event_code;

        /* the current time */
        uhd::time_spec_t c_time = _dev->get_time_now();
        uint64_t sync_secs = (uint64_t) c_time.get_full_secs();
        double sync_frac_of_secs = c_time.get_frac_secs();

        printf("----- ASYNC_MSG: event_code: %d, m_full_secs: %lld, m_frac_secs: %f, curr_full_secs: %llu, curr_frac_secs: %f----- \n", _event_code, (long long) _time_spec.get_full_secs(), _time_spec.get_frac_secs(), sync_secs, sync_frac_of_secs);
        fflush(stdout);
        msg.reset();
    }

protected:
    uhd::usrp::multi_usrp::sptr _dev;
    gruel::thread _amsg_thread;
    gr_msg_queue_sptr _msgq;
    bool _running;
};

/***********************************************************************
 * Make UHD Asynchronous Message Source
 **********************************************************************/
boost::shared_ptr<uhd_amsg_source> uhd_make_amsg_source(
    const uhd::device_addr_t &device_addr,
    gr_msg_queue_sptr msgq
){
    return boost::shared_ptr<uhd_amsg_source>(
        new uhd_amsg_source_impl(device_addr, msgq)
    );
}
