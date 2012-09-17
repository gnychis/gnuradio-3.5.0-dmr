/*
 * Copyright 2010-2011 Free Software Foundation, Inc.
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

#include <gr_uhd_usrp_sink.h>
#include <gr_io_signature.h>
#include <stdexcept>
#include <cstdio>
#include <boost/make_shared.hpp>

static const pmt::pmt_t SOB_KEY = pmt::pmt_string_to_symbol("tx_sob");
static const pmt::pmt_t EOB_KEY = pmt::pmt_string_to_symbol("tx_eob");
static const pmt::pmt_t TIME_KEY = pmt::pmt_string_to_symbol("tx_time");

#include <uhd/convert.hpp>
inline gr_io_signature_sptr args_to_io_sig(const uhd::stream_args_t &args){
    const size_t nchan = std::max<size_t>(args.channels.size(), 1);
    #ifdef GR_UHD_USE_STREAM_API
        const size_t size = uhd::convert::get_bytes_per_item(args.cpu_format);
    #else
        size_t size = 0;
        if (args.cpu_format == "fc32") size = 8;
        if (args.cpu_format == "sc16") size = 4;
    #endif
    return gr_make_io_signature(nchan, nchan, size);
}

/***********************************************************************
 * UHD Multi USRP Sink Impl
 **********************************************************************/
class uhd_usrp_sink_impl : public uhd_usrp_sink{
public:
    uhd_usrp_sink_impl(
        const uhd::device_addr_t &device_addr,
        const uhd::stream_args_t &stream_args
    ):
        gr_sync_block(
            "gr uhd usrp sink",
            args_to_io_sig(stream_args),
            gr_make_io_signature(0, 0, 0)
        ),
        _stream_args(stream_args),
        _nchan(std::max<size_t>(1, stream_args.channels.size()))
    {
        if (stream_args.cpu_format == "fc32") _type = boost::make_shared<uhd::io_type_t>(uhd::io_type_t::COMPLEX_FLOAT32);
        if (stream_args.cpu_format == "sc16") _type = boost::make_shared<uhd::io_type_t>(uhd::io_type_t::COMPLEX_INT16);
        _dev = uhd::usrp::multi_usrp::make(device_addr);

	uhd::device_addrs_t dev_addrs_vec = uhd::device::find(device_addr);
	uhd::device_addr_t dev_addr1 = dev_addrs_vec.at(0);
        if(dev_addr1.to_string().compare("type=usrp2,addr=192.168.20.2,name=,serial=E0R14VAUN") == 0) {
           printf("SLAVE CLOCK\n"); fflush(stdout);
           uhd::clock_config_t clock_config1;
           clock_config1.ref_source = uhd::clock_config_t::REF_MIMO;
           clock_config1.pps_source = uhd::clock_config_t::PPS_MIMO;
           this->set_clock_config(clock_config1, 0);
        }
        else {
           printf("Switching to the external clock!\n"); fflush(stdout);
           this->set_clock_config(uhd::clock_config_t::external(), 0);
           this->set_time_next_pps(uhd::time_spec_t(0.0));
	}
    }

    void set_subdev_spec(const std::string &spec, size_t mboard){
        return _dev->set_tx_subdev_spec(spec, mboard);
    }

    void set_samp_rate(double rate){
        _dev->set_tx_rate(rate);
        _sample_rate = this->get_samp_rate();
    }

    double get_samp_rate(void){
        return _dev->get_tx_rate();
    }

    uhd::meta_range_t get_samp_rates(void){
        #ifdef UHD_USRP_MULTI_USRP_GET_RATES_API
        return _dev->get_tx_rates();
        #else
        throw std::runtime_error("not implemented in this version");
        #endif
    }

    uhd::tune_result_t set_center_freq(
        const uhd::tune_request_t tune_request, size_t chan
    ){
	uhd::tune_request_t tune_req(tune_request.target_freq, 100e6/32);		// apurv
        return _dev->set_tx_freq(tune_req, chan);					// apurv
    }

    double get_center_freq(size_t chan){
        return _dev->get_tx_freq(chan);
    }

    uhd::freq_range_t get_freq_range(size_t chan){
        return _dev->get_tx_freq_range(chan);
    }

    void set_gain(double gain, size_t chan){
        return _dev->set_tx_gain(gain, chan);
    }

    void set_gain(double gain, const std::string &name, size_t chan){
        return _dev->set_tx_gain(gain, name, chan);
    }

    double get_gain(size_t chan){
        return _dev->get_tx_gain(chan);
    }

    double get_gain(const std::string &name, size_t chan){
        return _dev->get_tx_gain(name, chan);
    }

    std::vector<std::string> get_gain_names(size_t chan){
        return _dev->get_tx_gain_names(chan);
    }

    uhd::gain_range_t get_gain_range(size_t chan){
        return _dev->get_tx_gain_range(chan);
    }

    uhd::gain_range_t get_gain_range(const std::string &name, size_t chan){
        return _dev->get_tx_gain_range(name, chan);
    }

    void set_antenna(const std::string &ant, size_t chan){
        return _dev->set_tx_antenna(ant, chan);
    }

    std::string get_antenna(size_t chan){
        return _dev->get_tx_antenna(chan);
    }

    std::vector<std::string> get_antennas(size_t chan){
        return _dev->get_tx_antennas(chan);
    }

    void set_bandwidth(double bandwidth, size_t chan){
        return _dev->set_tx_bandwidth(bandwidth, chan);
    }

    void set_dc_offset(const std::complex<double> &offset, size_t chan){
        #ifdef UHD_USRP_MULTI_USRP_FRONTEND_CAL_API
        return _dev->set_tx_dc_offset(offset, chan);
        #else
        throw std::runtime_error("not implemented in this version");
        #endif
    }

    void set_iq_balance(const std::complex<double> &correction, size_t chan){
        #ifdef UHD_USRP_MULTI_USRP_FRONTEND_CAL_API
        return _dev->set_tx_iq_balance(correction, chan);
        #else
        throw std::runtime_error("not implemented in this version");
        #endif
    }

    uhd::sensor_value_t get_sensor(const std::string &name, size_t chan){
        return _dev->get_tx_sensor(name, chan);
    }

    std::vector<std::string> get_sensor_names(size_t chan){
        return _dev->get_tx_sensor_names(chan);
    }

    uhd::sensor_value_t get_mboard_sensor(const std::string &name, size_t mboard){
        return _dev->get_mboard_sensor(name, mboard);
    }

    std::vector<std::string> get_mboard_sensor_names(size_t mboard){
        return _dev->get_mboard_sensor_names(mboard);
    }

    void set_clock_config(const uhd::clock_config_t &clock_config, size_t mboard){
        return _dev->set_clock_config(clock_config, mboard);
    }

    void set_time_source(const std::string &source, const size_t mboard){
        #ifdef UHD_USRP_MULTI_USRP_REF_SOURCES_API
        return _dev->set_time_source(source, mboard);
        #else
        throw std::runtime_error("not implemented in this version");
        #endif
    }

    std::string get_time_source(const size_t mboard){
        #ifdef UHD_USRP_MULTI_USRP_REF_SOURCES_API
        return _dev->get_time_source(mboard);
        #else
        throw std::runtime_error("not implemented in this version");
        #endif
    }

    std::vector<std::string> get_time_sources(const size_t mboard){
        #ifdef UHD_USRP_MULTI_USRP_REF_SOURCES_API
        return _dev->get_time_sources(mboard);
        #else
        throw std::runtime_error("not implemented in this version");
        #endif
    }

    void set_clock_source(const std::string &source, const size_t mboard){
        #ifdef UHD_USRP_MULTI_USRP_REF_SOURCES_API
        return _dev->set_clock_source(source, mboard);
        #else
        throw std::runtime_error("not implemented in this version");
        #endif
    }

    std::string get_clock_source(const size_t mboard){
        #ifdef UHD_USRP_MULTI_USRP_REF_SOURCES_API
        return _dev->get_clock_source(mboard);
        #else
        throw std::runtime_error("not implemented in this version");
        #endif
    }

    std::vector<std::string> get_clock_sources(const size_t mboard){
        #ifdef UHD_USRP_MULTI_USRP_REF_SOURCES_API
        return _dev->get_clock_sources(mboard);
        #else
        throw std::runtime_error("not implemented in this version");
        #endif
    }

    double get_clock_rate(size_t mboard){
        return _dev->get_master_clock_rate(mboard);
    }

    void set_clock_rate(double rate, size_t mboard){
        return _dev->set_master_clock_rate(rate, mboard);
    }

    uhd::time_spec_t get_time_now(size_t mboard = 0){
        return _dev->get_time_now(mboard);
    }

    uhd::time_spec_t get_time_last_pps(size_t mboard){
        return _dev->get_time_last_pps(mboard);
    }

    void set_time_now(const uhd::time_spec_t &time_spec, size_t mboard){
        return _dev->set_time_now(time_spec, mboard);
    }

    void set_time_next_pps(const uhd::time_spec_t &time_spec){
        return _dev->set_time_next_pps(time_spec);
    }

    void set_time_unknown_pps(const uhd::time_spec_t &time_spec){
        return _dev->set_time_unknown_pps(time_spec);
    }

    void set_command_time(const uhd::time_spec_t &time_spec, size_t mboard){
        #ifdef UHD_USRP_MULTI_USRP_COMMAND_TIME_API
        return _dev->set_command_time(time_spec, mboard);
        #else
        throw std::runtime_error("not implemented in this version");
        #endif
    }

    void clear_command_time(size_t mboard){
        #ifdef UHD_USRP_MULTI_USRP_COMMAND_TIME_API
        return _dev->clear_command_time(mboard);
        #else
        throw std::runtime_error("not implemented in this version");
        #endif
    }

    uhd::usrp::dboard_iface::sptr get_dboard_iface(size_t chan){
        return _dev->get_tx_dboard_iface(chan);
    }

    uhd::usrp::multi_usrp::sptr get_device(void){
        return _dev;
    }

/***********************************************************************
 * Work
 **********************************************************************/
    int work(
        int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items
    ){
        int ninput_items = noutput_items; //cuz its a sync block

        //send a mid-burst packet with time spec
        _metadata.start_of_burst = false;
        _metadata.end_of_burst = false;

        //collect tags in this work()
        const uint64_t samp0_count = nitems_read(0);
        get_tags_in_range(_tags, 0, samp0_count, samp0_count + ninput_items);
        if (not _tags.empty()) this->tag_work(ninput_items);

        uint64_t o_sync_secs = (uint64_t) _metadata.time_spec.get_full_secs();
        double o_sync_frac_of_secs = _metadata.time_spec.get_frac_secs();

        #ifdef GR_UHD_USE_STREAM_API
        //send all ninput_items with metadata
        const size_t num_sent = _tx_stream->send(
            input_items, ninput_items, _metadata, 1.0
        );
        #else
        const size_t num_sent = _dev->get_device()->send(
            input_items, ninput_items, _metadata,
            *_type, uhd::device::SEND_MODE_FULL_BUFF, 1.0
        );
        #endif

        //increment the timespec by the number of samples sent
        _metadata.time_spec += uhd::time_spec_t(0, num_sent, _sample_rate);
        uint64_t sync_secs = (uint64_t) _metadata.time_spec.get_full_secs();
        double sync_frac_of_secs = _metadata.time_spec.get_frac_secs();

        /* current time */
        uhd::time_spec_t c_time = get_time_now();
        uint64_t c_sync_secs = (uint64_t) c_time.get_full_secs();
        double c_sync_frac_of_secs = c_time.get_frac_secs();

	printf("uhd_usrp_sink(tx) :: num_sent:::::::: %d, ninput_items: %d, sob: %d, eob: %d, o_sync_sec: %llu, o_sync_frac: %f, sync_sec: %llu, sync_frac: %f, c_sync_sec: %llu, c_sync_frac: %f\n", num_sent, ninput_items, _metadata.start_of_burst, _metadata.end_of_burst, o_sync_secs, o_sync_frac_of_secs, sync_secs, sync_frac_of_secs, c_sync_secs, c_sync_frac_of_secs); fflush(stdout);
        return num_sent;
    }

/***********************************************************************
 * Tag Work
 **********************************************************************/
    inline void tag_work(int &ninput_items){
	//the for loop below assumes tags sorted by count low -> high
	std::sort(_tags.begin(), _tags.end(), gr_tag_t::offset_compare);

	//extract absolute sample counts
	const gr_tag_t &tag0 = _tags.front();
	const uint64_t tag0_count = tag0.offset;
	const uint64_t samp0_count = this->nitems_read(0);

	printf("uhd_usrp_sink(tx) :: tag_work called, num_tags: %d, nitems_read: %ld, offset: %ld\n", _tags.size(), samp0_count, tag0_count); fflush(stdout);
	//only transmit nsamples from 0 to the first tag
	//this ensures that the next work starts on a tag
	if (samp0_count != tag0_count){
	    ninput_items = tag0_count - samp0_count;
	    printf("uhd_usrp_sink(tx) :: ninput_items: %ld\n", ninput_items); fflush(stdout);
	    return;
	}

	//time will not be set unless a time tag is found
	_metadata.has_time_spec = false;

	//process all of the tags found with the same count as tag0
	BOOST_FOREACH(const gr_tag_t &my_tag, _tags){
	    const uint64_t my_tag_count = my_tag.offset;
	    const pmt::pmt_t &key = my_tag.key;
	    const pmt::pmt_t &value = my_tag.value;

	    //determine how many samples to send...
	    //from zero until the next tag or end of work
	    if (my_tag_count != tag0_count){
		ninput_items = my_tag_count - samp0_count;
		printf("uhd_usrp_sink(tx) :: my_tag_count: %ld, ninput_items: %ld\n", my_tag_count, ninput_items); fflush(stdout);
		break;
	    }

	    //handle end of burst with a mini end of burst packet
	    else if (pmt::pmt_equal(key, EOB_KEY)){
		    _metadata.end_of_burst = pmt::pmt_to_bool(value);
		    printf("uhd_usrp_sink(tx) :: EOB_KEY\n"); fflush(stdout);
		    ninput_items = 1;
		    return;
	    }

	    //set the start of burst flag in the metadata
	    else if (pmt::pmt_equal(key, SOB_KEY)){
		    _metadata.start_of_burst = pmt::pmt_to_bool(value);
		    printf("uhd_usrp_sink(tx) :: SOB_KEY, offset: %llu\n", my_tag_count); fflush(stdout); 
	    }

	    //set the time specification in the metadata
	    else if (pmt::pmt_equal(key, TIME_KEY)){
		    _metadata.has_time_spec = true;
		    _metadata.time_spec = uhd::time_spec_t(
				    pmt::pmt_to_uint64(pmt_tuple_ref(value, 0)),
				    pmt::pmt_to_double(pmt_tuple_ref(value, 1)));

		    uhd::time_spec_t c_time = get_time_now();
		    uhd::time_spec_t m_time = _metadata.time_spec;

		    uint64_t sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
		    double sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));
		    printf("uhd_usrp_sink(tx) :: TIME_KEY, offset: %llu, sync_secs: %llu, frac_sec: %f, curr_full_sec: %lld, curr_frac_sec: %f, m_full_sec: %lld, m_frac_sec: %f\n", my_tag_count, sync_secs, sync_frac_of_secs, (long long) c_time.get_full_secs(), c_time.get_frac_secs(), (long long) m_time.get_full_secs(), m_time.get_frac_secs()); fflush(stdout);
		    /* apurv++ end */
	    }
	}
    }

    //Send an empty start-of-burst packet to begin streaming.
    //Set at a time in the near future to avoid late packets.
    bool start(void){
	//printf("uhd_usrp_sink(tx) :: start(), tx_freq: %f, rx_freq: %f\n", _dev->get_tx_freq(), _dev->get_rx_freq()); fflush(stdout);
        #ifdef GR_UHD_USE_STREAM_API
        _tx_stream = _dev->get_tx_stream(_stream_args);
        #endif

        _metadata.start_of_burst = true;
        _metadata.end_of_burst = false;
        _metadata.has_time_spec = _nchan > 1;
        _metadata.time_spec = get_time_now() + uhd::time_spec_t(0.01);

        #ifdef GR_UHD_USE_STREAM_API
        _tx_stream->send(
            gr_vector_const_void_star(_nchan), 0, _metadata, 1.0
        );
        #else
        _dev->get_device()->send(
            gr_vector_const_void_star(_nchan), 0, _metadata,
            *_type, uhd::device::SEND_MODE_ONE_PACKET, 1.0
        );
        #endif
        return true;
    }

    //Send an empty end-of-burst packet to end streaming.
    //Ending the burst avoids an underflow error on stop.
    bool stop(void){
	printf("uhd_usrp_sink(tx) :: stop()\n"); fflush(stdout);
        _metadata.start_of_burst = false;
        _metadata.end_of_burst = true;
        _metadata.has_time_spec = false;

        #ifdef GR_UHD_USE_STREAM_API
        _tx_stream->send(gr_vector_const_void_star(_nchan), 0, _metadata, 1.0);
        #else
        _dev->get_device()->send(
            gr_vector_const_void_star(_nchan), 0, _metadata,
            *_type, uhd::device::SEND_MODE_ONE_PACKET, 1.0
        );
        #endif
        return true;
    }

private:
    uhd::usrp::multi_usrp::sptr _dev;
    const uhd::stream_args_t _stream_args;
    boost::shared_ptr<uhd::io_type_t> _type;
    #ifdef GR_UHD_USE_STREAM_API
    uhd::tx_streamer::sptr _tx_stream;
    #endif
    size_t _nchan;
    uhd::tx_metadata_t _metadata;
    double _sample_rate;

    //stream tags related stuff
    std::vector<gr_tag_t> _tags;
};

/***********************************************************************
 * Make UHD Multi USRP Sink
 **********************************************************************/
boost::shared_ptr<uhd_usrp_sink> uhd_make_usrp_sink(
    const uhd::device_addr_t &device_addr,
    const uhd::io_type_t &io_type,
    size_t num_channels
){
    //fill in the streamer args
    uhd::stream_args_t stream_args;
    switch(io_type.tid){
    case uhd::io_type_t::COMPLEX_FLOAT32: stream_args.cpu_format = "fc32"; break;
    case uhd::io_type_t::COMPLEX_INT16: stream_args.cpu_format = "sc16"; break;
    default: throw std::runtime_error("only complex float and shorts known to work");
    }
    stream_args.otw_format = "sc16"; //only sc16 known to work
    for (size_t chan = 0; chan < num_channels; chan++)
        stream_args.channels.push_back(chan); //linear mapping

    return uhd_make_usrp_sink(device_addr, stream_args);
}

boost::shared_ptr<uhd_usrp_sink> uhd_make_usrp_sink(
    const uhd::device_addr_t &device_addr,
    const uhd::stream_args_t &stream_args
){
    return boost::shared_ptr<uhd_usrp_sink>(
        new uhd_usrp_sink_impl(device_addr, stream_args)
    );
}
