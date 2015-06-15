@ECHO OFF

SETLOCAL
  IF DEFINED HUB4COM GOTO DEFINED_HUB4COM
    SET HUB4COM=hub4com
  :DEFINED_HUB4COM

  PATH %~dp0;%PATH%

  SET MODE=C
  SET LINKPORT_WRITE_LIMIT=0
  SET PORT_NUM=0
  SET TAG=0
  SET LINK_TYPE=SERIAL
  SET TRACE_COMMENT=#

  :BEGIN_PARSE_OPTIONS
    IF "%~1" == "" GOTO END_PARSE_OPTIONS

    SET OPTION=%~1
    SHIFT /1

    IF /I "%OPTION%" == "--help" GOTO USAGE

    IF /I "%OPTION%" NEQ "--trace" GOTO END_OPTION_TRACE
      SET TRACE_COMMENT=
      GOTO BEGIN_PARSE_OPTIONS
    :END_OPTION_TRACE

    IF /I "%OPTION%" NEQ "--mode" GOTO END_OPTION_MODE
      SET ARG=%~1
      SHIFT /1

      IF /I "%ARG:~0,1%" NEQ "c" GOTO END_OPTION_MODE_CLIENT
        SET MODE=C
        GOTO BEGIN_PARSE_OPTIONS
      :END_OPTION_MODE_CLIENT
      IF /I "%ARG:~0,1%" NEQ "s" GOTO END_OPTION_MODE_SERVER
        SET MODE=S
        GOTO BEGIN_PARSE_OPTIONS
      :END_OPTION_MODE_SERVER
      GOTO USAGE
    :END_OPTION_MODE

    IF /I "%OPTION%" == "--baud"   GOTO BEGIN_OPTION_LC
    IF /I "%OPTION%" == "--data"   GOTO BEGIN_OPTION_LC
    IF /I "%OPTION%" == "--parity" GOTO BEGIN_OPTION_LC
    IF /I "%OPTION%" == "--stop"   GOTO BEGIN_OPTION_LC
    GOTO END_OPTION_LC
    :BEGIN_OPTION_LC
      SET VAL=%~1
      SHIFT /1
      IF /I "%VAL:~0,1%"=="d" SET VAL=c
      SET OPTIONS=%OPTIONS% %OPTION%=%VAL%
      GOTO BEGIN_PARSE_OPTIONS
    :END_OPTION_LC

    IF /I "%OPTION%" NEQ "--link-type" GOTO END_OPTION_LINK_TYPE
      SET LINK_TYPE=%~1
      SHIFT /1
      GOTO BEGIN_PARSE_OPTIONS
    :END_OPTION_LINK_TYPE

    IF /I "%OPTION%" NEQ "--secret" GOTO END_OPTION_SECRET
      SET SECRET=%~1
      SHIFT /1
      GOTO BEGIN_PARSE_OPTIONS
    :END_OPTION_SECRET

    IF "%OPTION:~0,2%" == "--" GOTO USAGE

    IF %PORT_NUM% GTR 0 GOTO END_LINKPORT
      SET /A PORT_NUM=%PORT_NUM%+1

      IF "%TRACE_COMMENT%" NEQ "" GOTO END_LINKPORT_TRACE
        SET LINKPORT_FILTERS=%LINKPORT_FILTERS% --create-filter=trace,linkport,COM
        SET LINKPORT_FILTERS_ADDED=TRUE
      :END_LINKPORT_TRACE

      IF "%SECRET%" == "" GOTO END_LINKPORT_SECRET
        SET LINKPORT_FILTERS=%LINKPORT_FILTERS% --create-filter=crypt,linkport,crypt:"--secret=\"%SECRET%\""
        SET LINKPORT_FILTERS_ADDED=TRUE

        IF "%TRACE_COMMENT%" NEQ "" GOTO END_LINKPORT_TRACE_RAW
          SET LINKPORT_FILTERS=%LINKPORT_FILTERS% --create-filter=trace,linkport,RAW
        :END_LINKPORT_TRACE_RAW
      :END_LINKPORT_SECRET

      IF "%LINKPORT_FILTERS_ADDED%" NEQ "TRUE" GOTO END_LINKPORT_FILTERS
        SET LINKPORT_FILTERS=%LINKPORT_FILTERS% --add-filters=0:linkport
      :END_LINKPORT_FILTERS

      IF /I "%LINK_TYPE%" NEQ "serial" GOTO END_LINK_TYPE_SERIAL
        SET OPTIONS=%OPTIONS% \\.\%OPTION%
        GOTO BEGIN_PARSE_OPTIONS
      :END_LINK_TYPE_SERIAL

      IF /I "%LINK_TYPE%" NEQ "tcp" GOTO END_LINK_TYPE_TCP
        SET OPTIONS=%OPTIONS% --use-driver=tcp *%OPTION% --use-driver=serial
        GOTO BEGIN_PARSE_OPTIONS
      :END_LINK_TYPE_TCP

      GOTO USAGE
    :END_LINKPORT

    IF "%MODE%" NEQ "C" GOTO END_MODE_CLIENT
      IF "%CLIENT_OPTIONS%" NEQ "" GOTO END_MODE_CLIENT
      SET CLIENT_OPTIONS=--load=\"%~f0\",BEGIN_CLIENT_FILTERS,END_CLIENT_FILTERS:%TRACE_COMMENT%
    :END_MODE_CLIENT

    IF "%MODE%" NEQ "S" GOTO END_MODE_SERVER
      IF "%SERVER_OPTIONS%" NEQ "" GOTO END_MODE_SERVER
      SET SERVER_OPTIONS=--load=\"%~f0\",BEGIN_SERVER_FILTERS,END_SERVER_FILTERS:%TRACE_COMMENT%
    :END_MODE_SERVER

    SET /A MUXA_PORT_NUM=%PORT_NUM%
    SET /A MUXB_PORT_NUM=%PORT_NUM%+1
    SET /A SUBPORT_NUM=%PORT_NUM%+2

    SET OPTIONS=%OPTIONS% --load=\"%~f0\",BEGIN_ADD_SUBPORT,END_ADD_SUBPORT:%OPTION%,%MODE%,%TAG%,%MUXA_PORT_NUM%,%MUXB_PORT_NUM%,%SUBPORT_NUM%,%TRACE_COMMENT%

    SET /A PORT_NUM=%PORT_NUM%+3
    SET /A TAG=%TAG%+1
    SET /A LINKPORT_WRITE_LIMIT=%LINKPORT_WRITE_LIMIT%+256

    GOTO BEGIN_PARSE_OPTIONS

  :END_PARSE_OPTIONS

  IF %PORT_NUM% LEQ 1 GOTO USAGE

  SET TC=:

  @ECHO ON
    "%HUB4COM%" %LINKPORT_FILTERS% %CLIENT_OPTIONS% %SERVER_OPTIONS% --write-limit=%LINKPORT_WRITE_LIMIT% %OPTIONS%
  @ECHO OFF
ENDLOCAL

GOTO END
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:USAGE

ECHO Usage:
ECHO     %0 [options] ^<linkport^> [options] ^<subport1^> [[options] ^<subport2^> ...]
ECHO.
ECHO Options:
ECHO     --trace               - enable trace output.
ECHO     --help                - show this help.
ECHO.
ECHO Linkport options:
ECHO     --link-type ^<t^>       - for subsequent port set type ^<t^> (default is
ECHO                             serial), where ^<t^> is serial or tcp.
ECHO     --secret=^<secret^>     - encrypt data with key created from ^<secret^>.
ECHO.
ECHO Subports options:
ECHO     --mode ^<m^>            - for subsequent ports set mode to ^<m^> (default is
ECHO                             client), where ^<m^> is c[lient] or s[erver].
ECHO.
ECHO Serial options:
ECHO     --baud ^<b^>            - for subsequent ports set baud rate to ^<b^> (default
ECHO                             is 19200), where ^<b^> is positive number or
ECHO                             d[efault].
ECHO     --data ^<d^>            - for subsequent ports set data bits to ^<d^> (default
ECHO                             is 8), where ^<d^> is positive number or d[efault].
ECHO     --parity ^<p^>          - for subsequent ports set parity to ^<p^> (default is
ECHO                             no), where ^<p^> is n[o], o[dd], e[ven], m[ark],
ECHO                             s[pace] or d[efault].
ECHO     --stop ^<s^>            - for subsequent ports set stop bits to ^<s^> (default
ECHO                             is 1), where ^<s^> is 1, 1.5, 2 or d[efault].
ECHO.
ECHO     The value d[efault] above means to use current port settings.
ECHO     The values of baud rate, data bits, parity and stop bits for subports are
ECHO     initial only and will be changed if client side subports created with
ECHO     com0com driver.

GOTO END
################################################################
BEGIN_CLIENT_FILTERS
  %%1%% --create-filter=trace,serialC,COM
  --create-filter=escparse,serialC,parse
  %%1%% --create-filter=trace,serialC,ExM
  --create-filter=pinmap,serialC,pinmap:--rts=cts --dtr=dsr
  --create-filter=linectl,serialC,lc:--br=local --lc=local
  %%1%% --create-filter=trace,serialC,CxB

  %%1%% --create-filter=trace,telnetC,muxB
  --create-filter=telnet,telnetC,telnet:--comport=client
  %%1%% --create-filter=trace,telnetC,TxM
  #--create-filter=purge,telnetC,purge
  --create-filter=pinmap,telnetC,pinmap:--rts=cts --dtr=dsr --break=break
  --create-filter=linectl,telnetC,lc:--br=remote --lc=remote
END_CLIENT_FILTERS
################################################################
BEGIN_SERVER_FILTERS
  %%1%% --create-filter=trace,serialS,COM
  --create-filter=escparse,serialS,parse
  %%1%% --create-filter=trace,serialS,ExM
  --create-filter=purge,serialS,purge
  --create-filter=pinmap,serialS,pinmap:--rts=cts --dtr=dsr --break=break
  --create-filter=linectl,serialS,lc:--br=remote --lc=remote
  %%1%% --create-filter=trace,serialS,CxB

  %%1%% --create-filter=trace,telnetS,muxB
  --create-filter=telnet,telnetS,telnet:--comport=server --suppress-echo=yes
  %%1%% --create-filter=trace,telnetS,BxM
  --create-filter=lsrmap,telnetS,lsrmap
  --create-filter=pinmap,telnetS,pinmap:--cts=cts --dsr=dsr --dcd=dcd --ring=ring
  --create-filter=linectl,telnetS,lc:--br=local --lc=local
END_SERVER_FILTERS
################################################################
BEGIN_ADD_SUBPORT
  #####################################################################################################
  #
  # %%1%% - subport
  # %%2%% - mode (C or S)
  # %%3%% - tag (0-254)
  #
  # [\\.\%%1%%]--(serial%%2%%)--(telnet%%2%%)--[%%1%%-muxB]--[%%1%%-muxA]--(tag%%3%%)--
  #                                                                                    \
  # [\\.\%%1%%]--(serial%%2%%)--(telnet%%2%%)--[%%1%%-muxB]--[%%1%%-muxA]--(tag%%3%%)--->--[linkport]
  #                                                                                    /
  # [\\.\%%1%%]--(serial%%2%%)--(telnet%%2%%)--[%%1%%-muxB]--[%%1%%-muxA]--(tag%%3%%)--
  #####################################################################################################

  --use-driver=connector

  %%1%%-muxA

  %%7%% --create-filter=trace,tag%%3%%,muxA
  --create-filter=tag,tag%%3%%:--tag=%%3%%
  %%7%% --create-filter=trace,tag%%3%%,TxL
  --add-filters=%%4%%:tag%%3%%
  --bi-route=0:%%4%%
  --no-default-fc-route=%%4%%:0

  %%1%%-muxB

  --bi-connect=%%1%%-muxA:%%1%%-muxB
  --add-filters=%%5%%:telnet%%2%%

  --use-driver=serial
  --octs=off
  --write-limit=256

  \\.\%%1%%

  --bi-route=%%5%%:%%6%%
  --add-filters=%%6%%:serial%%2%%
END_ADD_SUBPORT
################################################################
:END
