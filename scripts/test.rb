#! /usr/bin/env ruby

require 'orocos'

if ARGV.length != 3
    STDERR.puts "usage: test.rb host subject segment"
    exit 1
end

Orocos.initialize
Orocos.run 'vicon::Task' => 'vicon_driver' do

    driver = Orocos.get 'vicon_driver'

    driver.host = ARGV[0]
    driver.subject = ARGV[1]
    driver.segment = ARGV[2]

    driver.configure
    driver.start

    Orocos.watch driver

    driver.stop
end
