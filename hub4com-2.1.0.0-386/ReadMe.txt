               ================================
               HUB for communications (hub4com)
               ================================

INTRODUCTION
============

The HUB for communications (hub4com) is a Windows application and is a part of
the com0com project.

It allows to receive data and signals from one port, modify and send it to a
number of ports and vice versa.

In conjunction with the com0com driver the hub4com allows to
  - handle data and signals from a single real serial device by a number of
    different applications. For example, several applications can share data
    from one GPS device;
  - use real serial ports of remote computer like if they exist on the local
    computer.

The homepage for com0com project is http://com0com.sourceforge.net/.


TESTING
=======

  1. With the com0com driver create three virtual COM port pairs with
     port names CNCA0, CNCB0, CNCA1, CNCB1, CNCA2 and CNCB2 (see
     com0com's ReadMe.txt file for details).
  2. Start the hub4com.exe on CNCB0, CNCB1 and CNCB2 ports:

       hub4com --route=All:All \\.\CNCB0 \\.\CNCB1 \\.\CNCB2

  3. Start the HyperTerminal on CNCA0 port.
  4. Start the HyperTerminal on CNCA1 port.
  5. Start the HyperTerminal on CNCA2 port.
  6. The data typed to any HyperTerminal window should be printed on
     the others HyperTerminal windows.


EXAMPLES OF USAGE
=================

GPS hub
-------

You have a GPS device that connected to your computer via a phisical COM1
port and you'd like to handle its data by two GPS applications. You can
do it this way:

  1. With the com0com's Setup Command Prompt create COM5<->CNCB0 and
     COM6<->CNCB1 virtual COM port pairs (see com0com's ReadMe.txt for
     more info). For example:

       command> install 0 PortName=COM5 -
       command> install 1 PortName=COM6 -

  2. Start the hub4com.exe on COM1, CNCB0 and CNCB1 ports:

       hub4com \\.\COM1 \\.\CNCB0 \\.\CNCB1

     It will send data received from COM1 port to CNCB0 and CNCB1 ports
     and it will send data received from CNCB0 port to COM1 port.

  3. Start the GPS applications on COM5 and COM6 ports.


COM port to telnet redirector
-----------------------------

You have old TERM95.EXE application from the Norton Commander 5.0 for
MS-DOS and you'd like to use it to communicate with your.telnet.server
telnet server. You can do so this way:

  1. With the com0com's Setup Command Prompt create COM2<->CNCB0 virtual
     COM port pair (see com0com's ReadMe.txt for more info). For example:

       command> install 0 PortName=COM2 -

  2. Start the com2tcp.bat on CNCB0 port. For example:

       com2tcp --telnet \\.\CNCB0 your.telnet.server telnet

  3. Start the TERM95.EXE on COM2 port.

BTW: com2tcp.bat is a wrapper to hub4com.exe. It works very similar to
     the "COM port to TCP redirector" (com2tcp). It supports all
     com2tcp's options. If you feel that com2tcp is what you need but
     can't find any required functionality (for example RFC 2217 support)
     then try use hub4com instead.


RFC 2217 server (TCP to COM port redirector)
--------------------------------------------

You have a server computer with phisical COM1 port and you'd like to share it
through the network by the RFC 2217 "Telnet Com Port Control Option" protocol:

  1. Start the com2tcp-rfc2217.bat on COM1 port. For example:

       com2tcp-rfc2217 COM1 7000

  It will listen TCP/IP port 7000 for incaming connections and
  redirect them to COM1 port.


RFC 2217 client (COM port to TCP redirector)
--------------------------------------------

You have a server computer your.comport.server with phisical serial port
shared through the network by the RFC 2217 protocol (see above example) and
you'd like to use it on the client computer like a virtual serial port.

  1. With the com0com's Setup Command Prompt create COM5<->CNCB0 virtual
     COM port pair (see com0com's ReadMe.txt for more info). For example:

       command> install 0 PortName=COM5,EmuBR=yes -

  2. Start the com2tcp-rfc2217.bat on CNCB0 port. For example:

       com2tcp-rfc2217 \\.\CNCB0 your.comport.server 7000

  It will redirect virtual serial port COM5 on the second computer to the
  phisical serial port on the first computer.


CHAT server
-----------

You'd like by using computer with address your.computer.addr to allow up
to 5 users to chat by using ordinary telnet application.

  1. Start the hub4com.exe on the computer with address your.computer.addr:

       hub4com --load=,,_END_
          --create-filter=telnet
          --add-filters=All:telnet
          --route=All:All
          --use-driver=tcp
          *5555
          *5555
          *5555
          *5555
          *5555
          _END_

  2. Now users can join to the chat by connecting to port 5555 of your
     computer. For example:

       telnet your.computer.addr 5555


CVS pserver proxy
-----------------

You have a computer that has not access to the internet and you'd like to
allow it to access to hub4com's CVS repository by using proxy computer
with address your.computer.addr.

  1. Start the hub4com.exe on the computer with address your.computer.addr:

       hub4com.exe --use-driver=tcp /2401 com0com.cvs.sourceforge.net:2401

  2. On computer that has not access to the internet check out the hub4com
     souce code from the CVS repository:

       cvs -d:pserver:anonymous@your.computer.addr:/cvsroot/com0com login
       cvs -z3 -d:pserver:anonymous@your.computer.addr:/cvsroot/com0com co -P hub4com


CVS ssh to https proxy
----------------------

You have a computer that is behind firewall which block traffic on the SSH
ports used to access SourceForge CVS servers for developers. SourceForge
provides SSH access to CVS servers using port 443, to eliminate this problem
but CVSNT does not allow specifying the port for using the SSH. To eliminate
this problem (for <your-progect>, <developername> and <modulename>):

  1. Start the hub4com.exe:

       hub4com.exe --use-driver=tcp <your-progect>.cvs.sourceforge.net:443 --interface=localhost /22

  2. Check out the souce code from the CVS repository:

       cvs -z3 -d:ssh:<developername>@localhost:/cvsroot/<your-progect> co -P <modulename>


Multiplexing
------------

You have a client computer that has a serial port COM1 and a server
computer that has four serial ports COM1, COM2, COM3 and COM4. Both
computers linked each other by null-modem cable between their COM1 ports.
There are not any other connections between the computers. You'd like to
use COM2, COM3 and COM4 of the server computer like if they exist on the
client computer:

--------------------------------------            --------------------
|                    Client computer |            | Server computer  |
|                                    |            |                  |
| app1 -- COM2 (virtual) --          | null-modem |          -- COM2 ---
|                          \         |   cable    |         /        |
| app2 -- COM3 (virtual) --->-- COM1 -------------- COM1 --<--- COM3 ---
|                          /         |            |         \        |
| app3 -- COM4 (virtual) --          |            |          -- COM4 ---
--------------------------------------            --------------------
                             |<--------multiplexed------->|
                         |<-------------RFC 2217------------->|

  1. With the com0com's Setup Command Prompt on client computer create
     COM2<->CNCB0, COM3<->CNCB1 and COM4<->CNCB2 virtual serial port
     pairs (see com0com's ReadMe.txt for more info). For example:

       command> install 0 PortName=COM2,EmuBR=yes -
       command> install 1 PortName=COM3,EmuBR=yes -
       command> install 2 PortName=COM4,EmuBR=yes -

  2. Start the multiplexer.bat on the Server computer:

       multiplexer --baud 115200 COM1 --mode server CNCB1 CNCB2 CNCB3

  3. Start the multiplexer.bat on the Client computer:

       multiplexer --baud 115200 COM1 CNCB1 CNCB2 CNCB3


Encryption
----------

You have a server computer that has three serial ports COM2, COM3 and COM4.
You'd like to use COM2, COM3 and COM4 of the server computer like if they
exist on the client computer and use the server computer's TCP/IP port
serial.server.addr:5555 for this. Additionally you'd like to encrypt TCP/IP
traffic to protect the private data from others:

--------------------------------               ------------------------
|              Client computer |               | Server computer      |
|                              |               | (serial.server.addr) |
|                              |               |                      |
| app1 -- COM2 (virtual) --    |               |            -- COM2 -----
|                          \   |               | port      /          |
| app2 -- COM3 (virtual) --->-------TCP/IP-----> 5555-----<--- COM3 -----
|                          /   |               |           \          |
| app3 -- COM4 (virtual) --    |               |            -- COM4 -----
--------------------------------               ------------------------
                               |<--encrypted-->|
                             |<---multiplexed----------->|
                         |<--------RFC 2217----------------->|

  1. With the com0com's Setup Command Prompt on client computer create
     COM2<->CNCB0, COM3<->CNCB1 and COM4<->CNCB2 virtual serial port
     pairs (see com0com's ReadMe.txt for more info). For example:

       command> install 0 PortName=COM2,EmuBR=yes -
       command> install 1 PortName=COM3,EmuBR=yes -
       command> install 2 PortName=COM4,EmuBR=yes -

  2. Start the multiplexer.bat on the Server computer:

       multiplexer --secret "any secret phrase" --link-type tcp 5555 --mode server CNCB1 CNCB2 CNCB3

  3. Start the multiplexer.bat on the Client computer:

       multiplexer --secret "any secret phrase" --link-type tcp serial.server.addr:5555 CNCB1 CNCB2 CNCB3


FAQs & HOWTOs
=============

Q. The com2tcp.bat, com2tcp-rfc2217.bat and multiplexer.bat are a wrappers to
   the hub4com.exe. How to convert wrapper's command line to a file with
   arguments for hub4com.exe?
A. The file can be created by seting OPTIONS variable, for example:

   1. Convert multiplexer.bat's command line to the my.txt file:

      SET OPTIONS=--create-filter=trace,_CUT_THIS_LINE_
      multiplexer --baud 115200 COM1 CNCB1 CNCB2 CNCB3 > my.txt

   2. Cut line "--create-filter=trace,_CUT_THIS_LINE_" from my.txt file.

   Now the folloving two command lines are equal:

     multiplexer --baud 115200 COM1 CNCB1 CNCB2 CNCB3
     hub4com --load=my.txt,_BEGIN_,_END_

Q. We've been using and experimenting with hub4com and came to the conclusion
   that if you use a real com port in it, the real com port won't receive
   anything from the other loopback ports. Is there some trick to getting a
   physical com port to accept data with hub4com?
A. By default hub4com use CTS handshaking on output so if CTS state is OFF
   then no any data can be sent to the port. To disable CTS handshaking add
   --octs=off option before real com port.

Q. This is my config:
   A com0com  virtual null modem (COM3<>CNCB0) is used to get input data from
   the Windows "Generic Text Only" printer (configured on virtual COM4). After
   this I used hub4com to "split" the ASCII stream in 2 data paths: one stream
   goes to a dot matrix printer connected via Ethernet (hub4com creates a TCP
   client that connect to Jetdirect port 9100 of the printer) and the other one
   is used for a local TCP server on port 5555 (hub4com create a TCP server
   that will be used by a TCP client on networked PC to collect the same data
   printed on paper). This is the hub4com command-line:

     hub4com.exe --route=0:1,2 \\.\CNCB0 --use-driver=tcp *5555 *172.16.32.12:9100

   In this way it seems to me that everything works as desired: when the
   printer closes the TCP connection the hub4com immediately opens a new one.

   How to create a TCP connection when needed instead of using a "permanent"
   TCP connection w/o a risk to lose some bytes of data when hub4com (re)create
   the TCP connection to the printer?

   I want to be sure that the TCP client on networked PC continues to get data
   also in case that the printer is switched off (or disconnected from the
   network) and, at the same time, I want to be sure that the printer continues
   to print data also in case that the TCP client is switched off (or disconnected
   from the network). How to do it?

A. The following config script allows

    - create a TCP connection to the printer when needed instead of using a
      "permanent" TCP connection.
    - suspend a printing job till the job will cancelled by user or till
      connection to the printer or from TCP client on networked PC will
      established.
    - repeate each minute tries to create a TCP connection to the printer till
      there is a printing job.

    Copy and paste the following script to a command file (.bat) or to the console

    ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
    :
    : Set parameters
    :
    SET PRINTER_SERIAL_PORT=COM3
    SET H4C_SERIAL_PORT=CNCB0
    SET H4C_TCP_SERVER_PORT=5555
    SET H4C_TCP_CLIENT_ADDR=172.16.32.12
    :
    : Set required flow control on the printer's side
    :
    mode com4 octs=on odsr=on dtr=on rts=on
    :
    : Set config file path for hub4com to this bat-file or console
    :
    SET H4C_CONFIG_FILE="\"%~f0\""
    GOTO END_SET_H4C_CONFIG_FILE
    SET H4C_CONFIG_FILE=""
    :END_SET_H4C_CONFIG_FILE
    :
    : Start hub4com
    :
    hub4com --load=%H4C_CONFIG_FILE%,_BEGIN_H4C_,_END_H4C_:%H4C_SERIAL_PORT%,%H4C_TCP_SERVER_PORT%,%H4C_TCP_CLIENT_ADDR%
    :
    : Skip config for hub4com if something is wrong
    :
    GOTO END
    find "blah blah blah"
    :
    : Config for hub4com
    :
    _BEGIN_H4C_
    # Enable route data and signals
    #  - from SERIAL(0) to TCP(1) and TCP(2)
    #  - from TCP(1) to SERIAL(0)
    #  - from TCP(2) to SERIAL(0)
    # Enable route flow control
    #  - from TCP(1) and TCP(2) to SERIAL(0)
    #
    --bi-route=0:1,2
    #
    # The DTR should be in raised state on the SERIAL(0)
    # if TCP(1) or TCP(2) or both are in connected state
    # The DTR should be in not raised state on the SERIAL(0)
    # if TCP(1) and TCP(2) are in not connected state
    #
    --create-filter=pinmap:--dtr=connect
    --add-filters=0:pinmap
    #
    # (re)connect TCP(2) only when the DSR on the SERIAL(0)
    # is in raised state
    #
    --create-filter=pin2con:--connect=dsr
    --add-filters=0:pin2con
    #
    # Disable writing data to the SERIAL(0)
    #
    --write-limit=0
    #
    # Serial SERIAL(0)
    #
    \\.\%%1%%
    #
    # Restore write queue limit
    #
    --write-limit=256
    #
    --use-driver=tcp
    #
    # Server TCP(1)
    #
    *%%2%%
    #
    # Set write queue limit to a big enough value for TCP(2) to
    # minimize delays while connection procedure is in progress
    # (it's optional)
    # --write-limit=2048
    #
    # Do reconnect TCP(2) each minute till there is
    # a job for printing (the DSR on the SERIAL(0)
    # is in raised state)
    #
    --reconnect=60000
    #
    # Client TCP(2) to JetDirect server
    #
    %%3%%:9100
    #
    _END_H4C_
    find "blah blah blah"
    :END
    ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
    ::                          Press CTRL+C                          ::
    ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
