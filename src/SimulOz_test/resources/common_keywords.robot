*** Settings ***
Library           OperatingSystem
Library           Process

*** Keywords ***

Setup OzCore Test
  ${pgrep_rc}=  Run And Return Rc  pgrep -x OzCore
  Run Keyword If  ${pgrep_rc} == 0  Fail  msg=An OzCore process is already running ! Please kill it before running tests.
  Create Directory  ${TEST NAME}

Teardown OzCore Test
  Terminate All Processes  kill=True



