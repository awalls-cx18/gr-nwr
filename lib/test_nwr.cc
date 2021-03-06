/* -*- c++ -*- */
/*
 * Copyright (C) 2016  Andy Walls <awalls.cx18@gmail.com>
 *
 * This file was automatically generated by gr_modtool from GNU Radio
 *
 * This file was automatically generated from a template incorporating
 * data input by Andy Walls.
 * See http://www.gnu.org/licenses/gpl-faq.en.html#GPLOutput .
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <cppunit/TextTestRunner.h>
#include <cppunit/XmlOutputter.h>

#include <gnuradio/unittests.h>
#include "qa_nwr.h"
#include <iostream>
#include <fstream>

int
main (int argc, char **argv)
{
  CppUnit::TextTestRunner runner;
  std::ofstream xmlfile(get_unittest_path("nwr.xml").c_str());
  CppUnit::XmlOutputter *xmlout = new CppUnit::XmlOutputter(&runner.result(), xmlfile);

  runner.addTest(qa_nwr::suite());
  runner.setOutputter(xmlout);

  bool was_successful = runner.run("", false);

  return was_successful ? 0 : 1;
}
