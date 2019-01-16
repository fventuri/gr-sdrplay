/* -*- c++ -*- */

#define SDRPLAY_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "sdrplay_swig_doc.i"

%{
#include "sdrplay/rsp1_source.h"
#include "sdrplay/rsp1a_source.h"
#include "sdrplay/rsp2_source.h"
#include "sdrplay/rspduo_source.h"
%}


%include "sdrplay/rsp1_source.h"
GR_SWIG_BLOCK_MAGIC2(sdrplay, rsp1_source);
%include "sdrplay/rsp1a_source.h"
GR_SWIG_BLOCK_MAGIC2(sdrplay, rsp1a_source);
%include "sdrplay/rsp2_source.h"
GR_SWIG_BLOCK_MAGIC2(sdrplay, rsp2_source);
%include "sdrplay/rspduo_source.h"
GR_SWIG_BLOCK_MAGIC2(sdrplay, rspduo_source);
