*** Settings ***
Library           OperatingSystem
Library           Process
Test Teardown     Terminate All Processes

*** Variables ***
${MESSAGE}       Hoeing_test_started
${package}       oz440_gazebo
${logfolder}     /home/fanny/Tests/SimulOz_test/Log
${conffolder}    /home/fanny/OzCore/resources.local
${json}          /home/fanny/Tests/SimulOz_test/Tests/
${OZCORE_BINARY}  /home/fanny/OzCore/OzCore/build/Core/OzCore
${ROSLAUNCH_BINARY}  /opt/ros/kinetic/bin/roslaunch
${timeout}       10s

*** Test Cases ***
Oz Should Hoe Single Track Leek Configs
  [Template]  Oz Should Hoe Tracks
  field_config = LEEK_50_70_SQUARE
  field_config = LEEK_70_70_SQUARE
  field_config = LEEK_50_6_NO_STICK
  field_config = LEEK_50_8_GAP_1m
  field_config = LEEK_50_8_GAP_1m_QUINCONCE
  field_config = LEEK_50_8_GAP_3m
  field_config = LEEK_50_8_GAP_4m
  field_config = LEEK_50_8_GRASS
  field_config = LEEK_50_15_PACKS_GRASS
  field_config = LEEK_50_35_PACKS_GRASS

Oz Should Hoe Single Track Cabbage Configs
  [Template]  Oz Should Hoe Tracks
  field_config = CAB_70_70_SQUARE
  field_config = CAB_50_20_NO_STICK
  field_config = CAB_50_20_GAP_1m
  field_config = CAB_50_20_GAP_1m_QUINCONCE
  field_config = CAB_50_20_GAP_3m
  field_config = CAB_50_2_GAP_4m
  field_config = CAB_50_20_GRASS
  field_config = CAB_50_30_PACKS_GRASS
  field_config = CAB_50_50_PACKS_GRASS

*** Keywords ***

# field keywords

Oz Should Hoe Tracks
  [Arguments]  ${field_config}
  ${SimulOz successfully launched} =  Set Variable  false
  ${Roslaunch process ID} =  Launch roslaunch  ${package}  ${field_config}.launch
  ${SimulOz successfully launched} =  Wait for SimulOz to be launched  ${timeout}
  ${OzCore process ID} =  Launch OzCore  ${conffolder}  ${logfolder}  ${json}  ${field_config}
  Send Signal to process  SIGINT  ${Roslaunch process ID}  group=True
  Send Signal to process  SIGKILL  ${OzCore process ID}  group=True

*** Keywords ***

Launch OzCore
     [Arguments]   ${conffolder}  ${logfolder}  ${json}  ${field_config}
     ${OzCore process ID} =  Start Process  ${OZCORE_BINARY} --confFolder ${conffolder} --logFolder ${logfolder} --json ${json}/${field_config}/ozcore.json  shell=True  stdout=ozcore.out  stderr=ozcore.err
     [Return]   ${OzCore process ID}

Wait for SimulOz to be launched
     [Arguments]   ${timeout}
     TODO
     [Return]   ${SimulOz successfully launched}

Launch roslaunch
    [Arguments]   ${package}   ${launchfile}
    ${Roslaunch process ID} =  Start Process  ${ROSLAUNCH_BINARY} ${package} ${launchfile}  shell=True  stdout=roslaunch.out  stderr=roslaunch.err
    [Return]   ${Roslaunch process ID}