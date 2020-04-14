#include "ns3/automotive-module.h"
#include "ns3/traci-module.h"
#include "ns3/core-module.h"
#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/lte-v2x-helper.h"
#include "ns3/network-module.h"
#include "ns3/config-store.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include <ns3/cni-urbanmicrocell-propagation-loss-model.h>
#include <ns3/buildings-helper.h>
#include <ns3/spectrum-analyzer-helper.h>
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/phy-stats-calculator.h"
#include <ns3/multi-model-spectrum-channel.h>
#include <functional>
#include <stdlib.h>
#include <cfloat>
#include <sstream>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("v2v-cv2x-sandbox");

int
main (int argc, char *argv[])
{
  double baseline= 320.0;     // Baseline distance in meter (150m for urban, 320m for freeway)

  /*** 0.a App Options ***/
  bool verbose = true;
  bool realtime = false;
  bool sumo_gui = true;
  double sumo_updates = 0.01;
  bool send_cam = true;
  bool send_denm = true;
  bool asn = false;
  std::string sumo_folder = "src/automotive/examples/sumo-files/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo-files/map.sumo.cfg";
  double cam_intertime = 0.1;
  bool send_lon_lat = false;

  /*** 0.b LENA + V2X Options ***/
  uint16_t numberOfNodes;
  uint32_t nodeCounter = 0;

  double ueTxPower = 23.0;                // Transmission power in dBm
  double probResourceKeep = 0.0;          // Probability to select the previous resource again [0.0-0.8]
  uint32_t mcs = 20;                      // Modulation and Coding Scheme
  bool harqEnabled = false;               // Retransmission enabled
  bool adjacencyPscchPssch = true;        // Subchannelization scheme
  bool partialSensing = false;            // Partial sensing enabled (actual only partialSensing is false supported)
  uint16_t sizeSubchannel = 10;           // Number of RBs per subchannel
  uint16_t numSubchannel = 3;             // Number of subchannels per subframe
  uint16_t startRbSubchannel = 0;         // Index of first RB corresponding to subchannelization
  uint16_t pRsvp = 100;                   // Resource reservation interval
  uint16_t t1 = 4;                        // T1 value of selection window
  uint16_t t2 = 100;                      // T2 value of selection window
  uint16_t slBandwidth;                   // Sidelink bandwidth

  double simTime = 100;

  CommandLine cmd;

  /* Cmd Line option for application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("send-cam", "Enable car to send cam", send_cam);
  cmd.AddValue ("send-denm", "Enable car to send cam", send_denm);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("asn", "Use ASN.1 or plain-text to send message", asn);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("cam-intertime", "CAM dissemination inter-time [s]", cam_intertime);
  cmd.AddValue ("lonlat", "Send LonLat instead on XY", send_lon_lat);

  /* Cmd Line option for v2x */
  cmd.AddValue ("adjacencyPscchPssch", "Scheme for subchannelization", adjacencyPscchPssch);
  cmd.AddValue ("sizeSubchannel", "Number of RBs per Subchannel", sizeSubchannel);
  cmd.AddValue ("numSubchannel", "Number of Subchannels", numSubchannel);
  cmd.AddValue ("startRbSubchannel", "Index of first subchannel index", startRbSubchannel);
  cmd.AddValue ("T1", "T1 Value of Selection Window", t1);
  cmd.AddValue ("T2", "T2 Value of Selection Window", t2);
  cmd.AddValue ("mcs", "Modulation and Coding Scheme", mcs);
  cmd.AddValue ("pRsvp", "Resource Reservation Interval", pRsvp);
  cmd.AddValue ("probResourceKeep", "Probability for selecting previous resource again", probResourceKeep);
  cmd.AddValue ("baseline", "Distance in which messages are transmitted and must be received", baseline);

  cmd.AddValue("sim-time", "Total duration of the simulation [s])", simTime);

  cmd.Parse (argc, argv);

  if (verbose)
    {
      LogComponentEnable ("v2v-cv2x-sandbox", LOG_LEVEL_INFO);
      LogComponentEnable ("v2v-CAM-DENM-sender", LOG_LEVEL_INFO);
    }

  /*** 0.c V2X Configurations ***/
  /* Set the UEs power in dBm */
  Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (ueTxPower));
  Config::SetDefault ("ns3::LteUePhy::RsrpUeMeasThreshold", DoubleValue (-10.0));
  /* Enable V2X communication on PHY layer */
  Config::SetDefault ("ns3::LteUePhy::EnableV2x", BooleanValue (true));

  /* Set power */
  Config::SetDefault ("ns3::LteUePowerControl::Pcmax", DoubleValue (ueTxPower));
  Config::SetDefault ("ns3::LteUePowerControl::PsschTxPower", DoubleValue (ueTxPower));
  Config::SetDefault ("ns3::LteUePowerControl::PscchTxPower", DoubleValue (ueTxPower));

  if (adjacencyPscchPssch)
  {
      slBandwidth = sizeSubchannel * numSubchannel;
  }
  else
  {
      slBandwidth = (sizeSubchannel+2) * numSubchannel;
  }

  /* Configure for UE selected */
  Config::SetDefault ("ns3::LteUeMac::UlBandwidth", UintegerValue (slBandwidth));
  Config::SetDefault ("ns3::LteUeMac::EnableV2xHarq", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::LteUeMac::EnableAdjacencyPscchPssch", BooleanValue (adjacencyPscchPssch));
  Config::SetDefault ("ns3::LteUeMac::EnablePartialSensing", BooleanValue (partialSensing));
  Config::SetDefault ("ns3::LteUeMac::SlGrantMcs", UintegerValue (mcs));
  Config::SetDefault ("ns3::LteUeMac::SlSubchannelSize", UintegerValue (sizeSubchannel));
  Config::SetDefault ("ns3::LteUeMac::SlSubchannelNum", UintegerValue (numSubchannel));
  Config::SetDefault ("ns3::LteUeMac::SlStartRbSubchannel", UintegerValue (startRbSubchannel));
  Config::SetDefault ("ns3::LteUeMac::SlPrsvp", UintegerValue (pRsvp));
  Config::SetDefault ("ns3::LteUeMac::SlProbResourceKeep", DoubleValue (probResourceKeep));
  Config::SetDefault ("ns3::LteUeMac::SelectionWindowT1", UintegerValue (t1));
  Config::SetDefault ("ns3::LteUeMac::SelectionWindowT2", UintegerValue (t2));

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();

  /* Use the realtime scheduler of ns3 */
  if(realtime)
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /*** 0.d Read from the mob_trace the number of vehicles that will be created.
   *      The file should begin with something like:
   *      <!-- number of vehicles:2 -->
   *      The file must be included in the sumo-folder
  ***/
  std::string path = sumo_folder + mob_trace;
  std::ifstream infile (path);
  std::string num_client;
  /* Read the file*/
  if (infile.good())
    {
      getline(infile, num_client);
    }
  infile.close();
  /* Manipulate the string to get only the number of vehicles present */
  num_client.erase (0,24);
  num_client.erase (num_client.end ()-4,num_client.end ());
  numberOfNodes = std::stoi (num_client);

  ns3::Time simulationTime (ns3::Seconds(simTime));

  /*** 1. Create LTE objects   ***/
  Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  lteHelper->DisableNewEnbPhy(); // Disable eNBs for out-of-coverage modelling

  /* V2X */
  Ptr<LteV2xHelper> lteV2xHelper = CreateObject<LteV2xHelper> ();
  lteV2xHelper->SetLteHelper (lteHelper);

  /* Configure eNBs' antenna parameters before deploying them. */
  lteHelper->SetEnbAntennaModelType ("ns3::NistParabolic3dAntennaModel");
  lteHelper->SetAttribute ("UseSameUlDlPropagationCondition", BooleanValue(true));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", StringValue ("54990"));
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::CniUrbanmicrocellPropagationLossModel"));

  /*** 2. Create Internet and ipv4 helpers ***/
  InternetStackHelper internet;
  Ipv4StaticRoutingHelper ipv4RoutingHelper;

  /*** 3. Create containers for UEs and eNB ***/
  NodeContainer ueNodes;
  NodeContainer enbNodes;
  enbNodes.Create(1);
  ueNodes.Create(numberOfNodes);

  /*** 4. Create and install mobility (SUMO will be attached later) ***/
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(enbNodes);
  mobility.Install(ueNodes);
  /* Set the eNB to a fixed position */
  Ptr<MobilityModel> mobilityeNBn = enbNodes.Get (0)->GetObject<MobilityModel> ();
  mobilityeNBn->SetPosition (Vector (0, 0, 20.0)); // set eNB to fixed position - it is still disabled

  /*** 5. Install LTE Devices to the nodes + assign IP to UEs + manage buildings***/
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes); // If you don't do it, the simulation crashes

  /* Required to use NIST 3GPP model */
  BuildingsHelper::Install (ueNodes);
  BuildingsHelper::Install (enbNodes);
  BuildingsHelper::MakeMobilityModelConsistent ();

  lteHelper->SetAttribute("UseSidelink", BooleanValue (true));
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

  /* Install the IP stack on the UEs */
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));

  /* Assign IP address to UEs */
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      /* Set the default gateway for the UE */
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  /* attach devices */
  lteHelper->Attach (ueLteDevs);

  /* Create sidelink groups */
  std::vector<NetDeviceContainer> txGroups;
  txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueLteDevs, numberOfNodes);

  /* compute average number of receivers associated per transmitter and vice versa */
  std::map<uint32_t, uint32_t> txPerUeMap;
  std::map<uint32_t, uint32_t> groupsPerUe;
  std::vector<NetDeviceContainer>::iterator gIt;
  for(gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
      {
          uint32_t numDevs = gIt->GetN();

          uint32_t nId;

          for(uint32_t i=1; i< numDevs; i++)
              {
                  nId = gIt->Get(i)->GetNode()->GetId();
                  txPerUeMap[nId]++;
              }
      }

  std::map<uint32_t, uint32_t>::iterator mIt;
  for(mIt=txPerUeMap.begin(); mIt != txPerUeMap.end(); mIt++)
      {
          groupsPerUe [mIt->second]++;
      }

  // lteV2xHelper->PrintGroups (txGroups, log_connections);

  std::vector<uint32_t> groupL2Addresses;
  uint32_t groupL2Address = 0x00;
  std::vector<Ipv4Address> ipAddresses;
  Ipv4AddressGenerator::Init(Ipv4Address ("225.0.0.0"), Ipv4Mask ("255.0.0.0"));
  Ipv4Address clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
  NetDeviceContainer activeTxUes;

  for(gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
      {
          /* Create Sidelink bearers */
          NetDeviceContainer txUe ((*gIt).Get(0));
          activeTxUes.Add(txUe);
          NetDeviceContainer rxUes = lteV2xHelper->RemoveNetDevice ((*gIt), txUe.Get (0));
          Ptr<LteSlTft> tft = Create<LteSlTft> (LteSlTft::TRANSMIT, clientRespondersAddress, groupL2Address);
          lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), txUe, tft);
          tft = Create<LteSlTft> (LteSlTft::RECEIVE, clientRespondersAddress, groupL2Address);
          lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), rxUes, tft);

          /* store and increment addresses */
          groupL2Addresses.push_back (groupL2Address);
          ipAddresses.push_back (clientRespondersAddress);
          groupL2Address++;
          clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
      }

  /* Creating sidelink configuration */
  Ptr<LteUeRrcSl> ueSidelinkConfiguration = CreateObject<LteUeRrcSl>();
  ueSidelinkConfiguration->SetSlEnabled(true);
  ueSidelinkConfiguration->SetV2xEnabled(true);

  LteRrcSap::SlV2xPreconfiguration preconfiguration;
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.carrierFreq = 54890;
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.slBandwidth = slBandwidth;

  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.nbPools = 1;
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.nbPools = 1;

  SlV2xPreconfigPoolFactory pFactory;
  pFactory.SetHaveUeSelectedResourceConfig (true);
  pFactory.SetSlSubframe (std::bitset<20> (0xFFFFF));
  pFactory.SetAdjacencyPscchPssch (adjacencyPscchPssch);
  pFactory.SetSizeSubchannel (sizeSubchannel);
  pFactory.SetNumSubchannel (numSubchannel);
  pFactory.SetStartRbSubchannel (startRbSubchannel);
  pFactory.SetStartRbPscchPool (0);
  pFactory.SetDataTxP0 (-4);
  pFactory.SetDataTxAlpha (0.9);

  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0] = pFactory.CreatePool ();
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0] = pFactory.CreatePool ();
  ueSidelinkConfiguration->SetSlV2xPreconfiguration (preconfiguration);

  lteHelper->InstallSidelinkV2xConfiguration (ueLteDevs, ueSidelinkConfiguration);

  /*** 6. Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumo_gui));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue ("--verbose true"));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  /*** 7. Setup interface and application for dynamic nodes ***/
  CAMSenderHelper CamSenderHelper (9);
  appSampleHelper AppSampleHelper;

  CamSenderHelper.SetAttribute ("RealTime", BooleanValue (realtime));
  CamSenderHelper.SetAttribute ("ASN", BooleanValue (asn));
  CamSenderHelper.SetAttribute ("Model", StringValue ("cv2x"));

  AppSampleHelper.SetAttribute ("Client", PointerValue (sumoClient));
  AppSampleHelper.SetAttribute ("LonLat", BooleanValue (send_lon_lat));
  AppSampleHelper.SetAttribute ("ASN", BooleanValue (asn));
  AppSampleHelper.SetAttribute ("SendDenm", BooleanValue (send_denm));
  AppSampleHelper.SetAttribute ("SendCam", BooleanValue (send_cam));
  AppSampleHelper.SetAttribute ("CAMIntertime", DoubleValue (cam_intertime));
  AppSampleHelper.SetAttribute ("PrintSummary", BooleanValue (true));

  /* callback function for node creation */
  int i=0;
  std::function<Ptr<Node> ()> setupNewWifiNode = [&] () -> Ptr<Node>
    {
      if (nodeCounter >= ueNodes.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");

      Ptr<Node> includedNode = ueNodes.Get(nodeCounter);
      ++nodeCounter; // increment counter for next node

      /* Install Application */
      CamSenderHelper.SetAttribute ("IpAddr", Ipv4AddressValue(ipAddresses[i]));
      i++;

      ApplicationContainer CAMSenderApp = CamSenderHelper.Install (includedNode);
      ApplicationContainer AppSample = AppSampleHelper.Install (includedNode);

      CAMSenderApp.Start (Seconds (0.0));
      CAMSenderApp.Stop (simulationTime - Simulator::Now () - Seconds (0.1));
      AppSample.Start (Seconds (0.0));
      AppSample.Stop (simulationTime - Simulator::Now () - Seconds (0.1));

      return includedNode;
    };

  /* callback function for node shutdown */
  std::function<void (Ptr<Node>)> shutdownWifiNode = [] (Ptr<Node> exNode)
    {
      /* stop all applications */
      Ptr<CAMDENMSender> CAMSender_ = exNode->GetApplication(0)->GetObject<CAMDENMSender>();
      Ptr<appSample> appSample_ = exNode->GetApplication(0)->GetObject<appSample>();

      if(CAMSender_)
        CAMSender_->StopApplicationNow();
      if(appSample_)
        appSample_->StopApplicationNow();

       /* set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));// rand() for visualization purposes

      /* NOTE: further actions could be required for a safe shut down! */
    };

  /* start traci client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  /* To enable statistics collection of LTE module */
  //lteHelper->EnableTraces ();

  /*** 8. Start Simulation ***/
  Simulator::Stop (simulationTime);

  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
