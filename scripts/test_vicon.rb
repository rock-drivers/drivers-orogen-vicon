#! /usr/bin/env ruby

require 'orocos'
require 'ping'
include Orocos

if !ARGV[0]
    STDERR.puts "usage: test.rb host subject segment"
    exit 1
end

#if !Ping.pingecho(ARGV[0],1)
#    STDERR.puts "Host not reachable!"
#    exit 1
#end

#ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos.initialize

Orocos.run 'vicon::Task' => 'vicon_driver' do

    Orocos.log_all

    vicon = TaskContext.get 'vicon_driver'

    addr = ARGV[0].split(":")
    vicon.host = addr[0]
    if addr[1]
        vicon.port = addr[1].to_i
    end
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
