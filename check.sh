#!/bin/bash
#
# check.sh
#    Script to verify example builds
#
# Inputs:
#   $1 - Parameter for make
########################################################################
set -e
#set -o verbose
#set -o xtrace

cd examples/1.\ Basics/Device.Info
make clean
make   $1
make clean

cd ../Position.Velocity.Time
make clean
make   $1
make clean

cd ../../2.\ Configuration/Commands
make clean
make   $1
make clean

cd ../Constellations
make clean
make   $1
make clean

cd ../ElevationCNR
make clean
make   $1
make clean

cd ../RTK.Fix.Timeout
make clean
make   $1
make clean

cd ../RTK.DifferentialSourceType
make clean
make   $1
make clean

cd ../RTK.ReliabilityLevel
make clean
make   $1
make clean

cd ../SetFixInterval
make clean
make   $1
make clean

cd ../../3.\ Satellites/Satellites
make clean
make   $1
make clean

cd ../../4.\ Messages/Msg.Enable
make clean
make   $1
make clean

cd ../Msg.Subscribe
make clean
make   $1
make clean

cd ../../5.\ Features/Odometer
make clean
make   $1
make clean

cd ../PPS
make clean
make   $1
make clean

cd ../../6.\ Accuracy\ and\ Protection/Position.Errors
make clean
make   $1
make clean

cd ../Protection
make clean
make   $1
make clean

cd ../../7.\ Rover\,\ Base\,\ Survey/Set.Base.Rover.Mode
make clean
make   $1
make clean

cd ../Survey.Fixed.Mode
make clean
make   $1
make clean

cd ../Survey.Fixed.Mode.LLA
make clean
make   $1
make clean

cd ../Survey.In.Mode
make clean
make   $1
make clean

cd ../Rover.RTCM.Output
make clean
make   $1
make clean

cd ../../8.\ Resets/Reset.Device
make clean
make   $1
make clean

cd ../Stop.Start.Engine
make clean
make   $1
make clean

cd ../../9.\ NTRIP/NTRIP.Client
make clean
make   $1
make clean

cd ../NTRIP.Server
make clean
make   $1
make clean

cd ../NTRIP.Server.Survey.In
make clean
make   $1
make clean

cd ../../10.\ PPP HAS Corrections/HAS_E6_Example
make clean
make   $1
make clean

cd ../../12.\ NAVMODE/NAVMODE_Example
make clean
make   $1
make clean

# Return to origin directory
cd ../../..
