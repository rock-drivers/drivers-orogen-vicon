#! /usr/bin/env ruby

require 'orocos'
include Orocos

if !ARGV[0]
    STDERR.puts "usage: test.rb host subject segment"
    exit 1
end

Orocos.initialize
Orocos.run 'vicon::Task' => 'vicon_driver' do

    driver = Orocos.get 'vicon_driver'

    addr = ARGV[0].split(":")
    driver.host = addr[0]
    if addr[1]
        driver.port = addr[1].to_i
    end
    driver.subject = ARGV[1]
    driver.segment = ARGV[2]

    driver.configure
    driver.start

    reader = driver.pose_samples.reader

    loop do
	if sample = reader.read
	    puts "%s %s %s" % [sample.position.data[0], sample.position.data[1], sample.position.data[2]]
	end
	sleep 0.01
    end

    driver.stop
end

