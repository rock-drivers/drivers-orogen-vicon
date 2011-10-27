#! /usr/bin/env ruby

require 'orocos'
include Orocos

if !ARGV[0]
    STDERR.puts "usage: test.rb host subject segment"
    exit 1
end

ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos.initialize

Orocos.run 'test_vicon' do

    Orocos.log_all

    vicon = TaskContext.get 'Task'

    vicon.host = ARGV[0]
    vicon.subject = ARGV[1]
    vicon.segment = ARGV[2]

    vicon.configure
    vicon.start

    reader = vicon.pose_samples.reader

    while true
        if sample = reader.read
	        puts "%s %s %s" % [sample.position.x, sample.position.y, sample.position.z]
        end
        sleep 0.01
    end

    vicon.stop
end
