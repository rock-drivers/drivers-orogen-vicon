#! /usr/bin/env ruby

require 'orocos'
include Orocos

if !ARGV[0]
    STDERR.puts "usage: test.rb host subject segment"
    exit 1
end

ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos.initialize

Orocos::Process.spawn 'test_vicon' do |p|
    driver = p.task 'Task'
    Orocos.log_all_ports

    driver.host = ARGV[0]
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

