/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/athstats-helper.h"
#include "ns3/internet-module.h"

#include "ns3/point-to-point-module.h"
#include "ns3/csma-module.h"

#include <iostream>

using namespace ns3;

static bool g_verbose = false;

void
DevTxTrace (std::string context, Ptr<const Packet> p)
{
  if (g_verbose)
    {
      std::cout << " TX p: " << *p << std::endl;
    }
}
void
DevRxTrace (std::string context, Ptr<const Packet> p)
{
  if (g_verbose)
    {
      std::cout << " RX p: " << *p << std::endl;
    }
}
void
PhyRxOkTrace (std::string context, Ptr<const Packet> packet, double snr, WifiMode mode, enum WifiPreamble preamble)
{
  if (g_verbose)
    {
      std::cout << "PHYRXOK mode=" << mode << " snr=" << snr << " " << *packet << std::endl;
    }
}
void
PhyRxErrorTrace (std::string context, Ptr<const Packet> packet, double snr)
{
  if (g_verbose)
    {
      std::cout << "PHYRXERROR snr=" << snr << " " << *packet << std::endl;
    }
}
void
PhyTxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower)
{
  if (g_verbose)
    {
      std::cout << "PHYTX mode=" << mode << " " << *packet << std::endl;
    }
}

void
PhyStateTrace (std::string context, Time start, Time duration, enum WifiPhy::State state)
{
  if (g_verbose)
    {
      std::cout << " state=" << state << " start=" << start << " duration=" << duration << std::endl;
    }
}

static void
SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

static Vector
GetPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

static void 
AdvancePosition (Ptr<Node> node) 
{
  Vector pos = GetPosition (node);
  pos.x += 1.0;
  if (pos.x >= 151.0) 
    {
      return;
    }
  SetPosition (node, pos);

  if (g_verbose)
    {
      //std::cout << "x="<<pos.x << std::endl;
    }
  Simulator::Schedule (Seconds (1.0), &AdvancePosition, node);
}

static void 
DecreasePosition (Ptr<Node> node) 
{
  Vector pos = GetPosition (node);
  pos.x -= 1.0;
  if (pos.x <= -151.0) 
    {
      return;
    }
  SetPosition (node, pos);

  if (g_verbose)
    {
      //std::cout << "x="<<pos.x << std::endl;
    }
  Simulator::Schedule (Seconds (1.0), &DecreasePosition, node);
}

int main (int argc, char *argv[])
{
  std::string model("a");
  std::string proto("udp");
  std::string data_rate("54Mbps");
  std::string phyMode ("DsssRate11Mbps");//Max speed for 802.11b
  std::string model_string;

  CommandLine cmd;
  cmd.AddValue ("verbose", "Print trace information if true", g_verbose);
  cmd.AddValue ("standard", "Use a 802.11 A/B/G", model);
  cmd.AddValue ("protocol", "Use Either udp or tcp", proto);
  cmd.Parse (argc, argv);

  Packet::EnablePrinting ();

  // enable rts cts all the time.
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("0"));
  // disable fragmentation
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));

  WifiHelper wifi = WifiHelper::Default ();
  MobilityHelper mobility;
  NodeContainer stas;
  NodeContainer ap;
  NetDeviceContainer staDevs,apDevs;
  Time interPacketInterval;

  if(model[0] == 'a')
  {
        wifi.SetStandard(WIFI_PHY_STANDARD_80211a);
        interPacketInterval = Seconds (0.00015);// a/g speeds
        model_string = "q2_a";
        data_rate = "54Mbps";
  }
  else if(model[0] == 'b')
  {
        wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
        interPacketInterval = Seconds (0.00074);// b speeds
        model_string = "q2_b";
        data_rate = "11Mbps";
  }
  else if(model[0] == 'g')
  {
        wifi.SetStandard(WIFI_PHY_STANDARD_80211g);
        interPacketInterval = Seconds (0.00015);// a/g speeds
        model_string = "q2_g";
        data_rate = "54Mbps";
  }
  else
  {
        std::cout << "Invalid 802.11 standard , a/b/g accepted, given:" << model << std::endl;
  }
  

  stas.Create (2);
  ap.Create (1);

  //Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  //positionAlloc -> Add(Vector(-141.0,5.0,0.0));
  //positionAlloc -> Add(Vector(0.0,0.0,0.0));
  //mobility.SetPositionAllocator (positionAlloc);
  

  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  Ssid ssid = Ssid ("wifi-default");

  // setup stas.
  wifiMac.SetType ("ns3::StaWifiMac",
                   "Ssid", SsidValue (ssid),
                   "ActiveProbing", BooleanValue (false));
  staDevs = wifi.Install (wifiPhy, wifiMac, stas);
  // setup ap.
  wifiMac.SetType ("ns3::ApWifiMac",
                   "Ssid", SsidValue (ssid));
  

  apDevs = wifi.Install (wifiPhy, wifiMac, ap);

  wifiPhy.EnablePcapAll(model_string,false);

  // mobility.
  mobility.Install (stas);
  mobility.Install (ap);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  SetPosition (stas.Get(0), Vector (-150.0,10.0,0.0));
  SetPosition (stas.Get(1), Vector (180.0,-10.0,0.0));
  SetPosition (ap.Get(0), Vector (0.0,0.0,0.0));
  
  Simulator::Schedule (Seconds (1.0), &AdvancePosition, stas.Get (0));
  Simulator::Schedule (Seconds (100.1), &DecreasePosition, stas.Get (1));
  
          InternetStackHelper internet;
          internet.Install (ap);
          internet.Install (stas);
         

         Ipv4AddressHelper address;
         address.SetBase ("10.1.3.0", "255.255.255.0");
         Ipv4InterfaceContainer apConnection = address.Assign(apDevs);
         Ipv4InterfaceContainer staConnection = address.Assign(staDevs);
         
         Ipv4Address serverAddress = Ipv4Address(apConnection.GetAddress(0,0));
         //Ipv4Address clientAddress = Ipv4Address(staConnection.GetAddress(0,0));

        if(proto[0] == 'u')
        {
                 //create server
                 uint16_t port = 9;
                 UdpServerHelper server (port);
                 ApplicationContainer apps = server.Install(ap.Get(0));

                 apps.Start (Seconds (0.1));
                 apps.Stop (Seconds (320.0));

                 //create client
                 uint32_t MaxPacketSize = 1024;
                 uint32_t maxPacketCount = 99999999;
                 UdpClientHelper client1 (serverAddress, port);
                 client1.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
                 client1.SetAttribute ("Interval", TimeValue (interPacketInterval));
                 client1.SetAttribute ("PacketSize", UintegerValue (MaxPacketSize));
                 
                 client1.Install (stas.Get (0));
                 apps = client1.Install (stas.Get (1));
                 apps.Start (Seconds (0.0));
                 apps.Stop (Seconds (320.0));
                
                 Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/State", MakeCallback (&PhyStateTrace));
        }
        else if(proto[0] == 't')
        {
               ApplicationContainer serverApps,sink1App;
	        uint16_t port = 50000;
	        Address apLocalAddress (InetSocketAddress (Ipv4Address::GetAny (), port));
	        PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", apLocalAddress);
	        sink1App = packetSinkHelper.Install (stas.Get (0));
	        sink1App.Start (Seconds (0.0));
	        sink1App.Stop (Seconds (320.0));

	        OnOffHelper onoff ("ns3::TcpSocketFactory",Ipv4Address::GetAny ());
	        onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
	        onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
	        onoff.SetAttribute ("PacketSize", UintegerValue (1024));
	        onoff.SetAttribute ("DataRate", DataRateValue (data_rate));
	        ApplicationContainer apps;

	        AddressValue remoteAddress (InetSocketAddress (staConnection.GetAddress(0), port));
	        onoff.SetAttribute ("Remote", remoteAddress);
	        apps.Add (onoff.Install (ap.Get (0)));
	        apps.Start (Seconds (0.0));
	        apps.Stop (Seconds (320.0));
               
               Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/Tx", MakeCallback (&PhyTxTrace));  
        }
        else
        {
                std::cout << "Unrecognized protocol , use either udp or tcp, given:"<<proto<<std::endl;
        }

  Simulator::Stop (Seconds (262.0));
  
  Simulator::Run ();

  Simulator::Destroy ();

  return 0;
}
