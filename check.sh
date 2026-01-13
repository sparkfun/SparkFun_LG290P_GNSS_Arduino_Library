#!/bin/bash
#
# check.sh
#    Script to verify example builds
########################################################################
set -e
#set -o verbose
#set -o xtrace

cd examples/1.\ Basics/Device.Info
make clean
make
make clean

cd ../Position.Velocity.Time
make clean
make
make clean

cd ../../2.\ Configuration/Commands
make clean
make
make clean

cd ../Constellations
make clean
make
make clean

cd ../ElevationCNR
make clean
make
make clean

cd ../RTK.Fix.Timeout
make clean
make
make clean

cd ../SetFixInterval
make clean
make
make clean

cd ../../3.\ Satellites/Satellites
make clean
make
make clean

cd ../../4.\ Messages/Msg.Enable
make clean
make
make clean

cd ../Msg.Subscribe
make clean
make
make clean

cd ../../5.\ Features/Odometer
make clean
make
make clean

cd ../PPS
make clean
make
make clean

cd ../../6.\ Accuracy\ and\ Protection/Position.Errors
make clean
make
make clean

cd ../Protection
make clean
make
make clean

cd ../../7.\ Rover\,\ Base\,\ Survey/Set.Base.Rover.Mode
make clean
make
make clean

cd ../Survey.Fixed.Mode
make clean
make
make clean

cd ../Survey.Fixed.Mode.LLA
make clean
make
make clean

cd ../Survey.In.Mode
make clean
make
make clean

cd ../../8.\ Resets/Reset.Device
make clean
make
make clean

cd ../Stop.Start.Engine
make clean
make
make clean

cd ../../9.\ NTRIP/NTRIP.Client
make clean
make
make clean

cd ../NTRIP.Server
make clean
make
make clean

cd ../NTRIP.Server.Survey.In
make clean
make
make clean

cd ../../10.\ PPP HAS Corrections/HAS_E6_Example

# Return to origin directory
cd ../../..
