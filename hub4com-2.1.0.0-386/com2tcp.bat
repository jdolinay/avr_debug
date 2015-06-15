@ECHO OFF

SETLOCAL
  IF DEFINED HUB4COM GOTO DEFINED_HUB4COM
    SET HUB4COM=hub4com
  :DEFINED_HUB4COM

  PATH %~dp0;%PATH%

  :SET PARAMS=%PARAMS% --create-filter=trace

  SET CF_PIN2CON=--create-filter=pin2con
  SET AF_PIN2CON=--add-filters=0:pin2con
  SET RECONNECT=1000

  :BEGIN_PARSE_OPTIONS
    SET OPTION=%~1
    IF NOT "%OPTION:~0,2%" == "--" GOTO END_PARSE_OPTIONS
    SHIFT /1

    IF /I "%OPTION%" == "--help" GOTO USAGE

    IF /I "%OPTION%" NEQ "--telnet" GOTO END_OPTION_TELNET
      SET CF_TELNET=--create-filter=telnet
      SET AF_TELNET=--add-filters=1:telnet
      GOTO BEGIN_PARSE_OPTIONS
    :END_OPTION_TELNET

    IF /I "%OPTION%" NEQ "--terminal" GOTO END_OPTION_TERMINAL
      SET CF_TELNET_OPTIONS="--terminal=\"%~1\""
      SHIFT /1
      GOTO BEGIN_PARSE_OPTIONS
    :END_OPTION_TERMINAL

    IF /I "%OPTION%" NEQ "--awak-seq" GOTO END_OPTION_AWAK_SEQ
      SET CF_AWAK_SEQ="--create-filter=awakseq:--awak-seq=\"%~1\""
      SET AF_AWAK_SEQ=--add-filters=0:awakseq
      SET RECONNECT=d
      SHIFT /1
      GOTO BEGIN_PARSE_OPTIONS
    :END_OPTION_AWAK_SEQ

    IF /I "%OPTION%" NEQ "--ignore-dsr" GOTO END_OPTION_IGNORE_DSR
      SET CF_PIN2CON=
      SET AF_PIN2CON=
      SET PERMANENT=*
      SET RECONNECT=d
      GOTO BEGIN_PARSE_OPTIONS
    :END_OPTION_IGNORE_DSR

    IF /I "%OPTION%" NEQ "--connect-dtr" GOTO END_OPTION_CONNECT_DTR
      SET CF_PINMAP=--create-filter=pinmap:"--dtr=connect"
      SET AF_PINMAP=--add-filters=0:pinmap
      GOTO BEGIN_PARSE_OPTIONS
    :END_OPTION_CONNECT_DTR

    IF /I "%OPTION%" NEQ "--interface" GOTO END_OPTION_INTERFACE
      SET OPTIONS=%OPTIONS% --interface=%~1
      SHIFT /1
      GOTO BEGIN_PARSE_OPTIONS
    :END_OPTION_INTERFACE

    IF /I "%OPTION%" == "--baud"   GOTO BEGIN_OPTION_LC
    IF /I "%OPTION%" == "--data"   GOTO BEGIN_OPTION_LC
    IF /I "%OPTION%" == "--parity" GOTO BEGIN_OPTION_LC
    IF /I "%OPTION%" == "--stop"   GOTO BEGIN_OPTION_LC
    GOTO END_OPTION_LC
    :BEGIN_OPTION_LC
      SET VAL=%~1
      SHIFT /1
      IF /I "%VAL:~0,1%" == "d" SET VAL=c
      SET OPTIONS=%OPTIONS% %OPTION%=%VAL%
      GOTO BEGIN_PARSE_OPTIONS
    :END_OPTION_LC

    GOTO USAGE
  :END_PARSE_OPTIONS

  :BEGIN_PARSE_ARGS
    IF "%~1" == "" GOTO USAGE
    SET COMPORT=%~1
    SHIFT /1

    IF "%~1" == "" GOTO USAGE
    SET TCP=%PERMANENT%%~1
    SHIFT /1

    IF "%~1" == "" GOTO END_PARSE_ARGS
    SET TCP=%TCP%:%~1
    SHIFT /1

    IF NOT "%~1" == "" GOTO USAGE
  :END_PARSE_ARGS

  IF "%CF_TELNET_OPTIONS%" == "" GOTO END_ADD_CF_TELNET_OPTIONS
  IF "%CF_TELNET%" == "" GOTO END_ADD_CF_TELNET_OPTIONS
    SET CF_TELNET=%CF_TELNET%:%CF_TELNET_OPTIONS%
  :END_ADD_CF_TELNET_OPTIONS

  SET PARAMS=%PARAMS% %CF_PINMAP% %AF_PINMAP%
  SET PARAMS=%PARAMS% %CF_PIN2CON% %AF_PIN2CON%
  SET PARAMS=%PARAMS% %CF_AWAK_SEQ% %AF_AWAK_SEQ%
  SET PARAMS=%PARAMS% %CF_TELNET% %AF_TELNET%

  @ECHO ON
    "%HUB4COM%" %OPTIONS% %PARAMS% "%COMPORT%" --use-driver=tcp --reconnect=%RECONNECT% "%TCP%"
  @ECHO OFF
ENDLOCAL

GOTO END
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:USAGE

ECHO Usage (client mode):
ECHO     %0 [options] \\.\^<com port^> ^<host addr^> ^<host port^>
ECHO.
ECHO Usage (server mode):
ECHO     %0 [options] \\.\^<com port^> ^<listen port^>
ECHO.
ECHO Common options:
ECHO     --telnet              - use Telnet protocol.
ECHO     --terminal ^<type^>     - use terminal ^<type^> (RFC 1091).
ECHO     --help                - show this help.
ECHO.
ECHO COM port options:
ECHO     --baud ^<b^>            - set baud rate to ^<b^> (default is 19200),
ECHO                             where ^<b^> is positive number or d[efault].
ECHO     --data ^<d^>            - set data bits to ^<d^> (default is 8), where ^<d^> is
ECHO                             positive number or d[efault].
ECHO     --parity ^<p^>          - set parity to ^<p^> (default is no), where ^<p^> is
ECHO                             n[o], o[dd], e[ven], m[ark], s[pace] or d[efault].
ECHO     --stop ^<s^>            - set stop bits to ^<s^> (default is 1), where ^<s^> is
ECHO                             1, 1.5, 2 or d[efault].
ECHO     --ignore-dsr          - ignore DSR state (do not wait DSR to be ON before
ECHO                             connecting to host, do not close connection after
ECHO                             DSR is OFF and do not ignore any bytes received
ECHO                             while DSR is OFF).
ECHO     --connect-dtr         - set DTR to ON/OFF on opening/closing connection to
ECHO                             host.
ECHO.
ECHO     The value d[efault] above means to use current COM port settings.
ECHO.
ECHO Client mode options:
ECHO     --awak-seq ^<sequence^> - wait for awakening ^<sequence^> from com port
ECHO                             before connecting to host. All data before
ECHO                             ^<sequence^> and ^<sequence^> itself will not be sent.
ECHO     --interface ^<if^>      - use interface ^<if^> for connecting.
ECHO.
ECHO Server mode options:
ECHO     --interface ^<if^>      - use interface ^<if^> for listening.

GOTO END
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:END
