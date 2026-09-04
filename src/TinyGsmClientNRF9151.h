/**
 * @file       TinyGsmClientNRF9151.h
 * @author     FOSSA Systems
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2016 Volodymyr Shymanskyy
 * @date       Sep 2026
 *
 * Modem driver for the Nordic Semiconductor nRF9151 SiP running the
 * "Serial LTE Modem" (SLM) sample/firmware, which exposes the cellular
 * modem over UART using standard 3GPP AT commands plus a set of Nordic
 * proprietary `AT#X...` commands for sockets, native TLS and device
 * management.
 *
 * The driver is aimed at native-TLS TCP sockets (for example MQTT and HTTP
 * clients), with up to two simultaneous connections (see TINY_GSM_MUX_COUNT
 * below). Plain (non-TLS) sockets are provided so that the TinyGsmTCP<>
 * interface is complete and symmetric with the secure client.
 *
 * Notes:
 *  - This class assumes the *caller* serializes all access to the sockets
 *    it creates (e.g. via an external mutex) so that "select socket, then
 *    operate on it" sequences are never interleaved by another thread/task.
 *    No locking is done inside this class.
 */

#ifndef SRC_TINYGSMCLIENTNRF9151_H_
#define SRC_TINYGSMCLIENTNRF9151_H_
// #pragma message("TinyGSM:  TinyGsmClientNRF9151")

// #define TINY_GSM_DEBUG Serial

// Two simultaneous sockets are supported, e.g. one MQTT-over-TLS connection
// and one HTTP-over-TLS connection.
#define TINY_GSM_MUX_COUNT 2
#define TINY_GSM_BUFFER_READ_AND_CHECK_SIZE
#ifdef AT_NL
#undef AT_NL
#endif
#define AT_NL "\r\n"

#ifdef MODEM_MANUFACTURER
#undef MODEM_MANUFACTURER
#endif
#define MODEM_MANUFACTURER "Nordic Semiconductor"

#ifdef MODEM_MODEL
#undef MODEM_MODEL
#endif
#define MODEM_MODEL "nRF9151"

#include "TinyGsmModem.tpp"
#include "TinyGsmTCP.tpp"
#include "TinyGsmSSL.tpp"
#include "TinyGsmGPRS.tpp"

enum NRF9151RegStatus {
  REG_NO_RESULT    = -1,
  REG_UNREGISTERED = 0,
  REG_SEARCHING    = 2,
  REG_DENIED       = 3,
  REG_OK_HOME      = 1,
  REG_OK_ROAMING   = 5,
  REG_UNKNOWN      = 4,
};

class TinyGsmNRF9151 : public TinyGsmModem<TinyGsmNRF9151>,
                       public TinyGsmGPRS<TinyGsmNRF9151>,
                       public TinyGsmTCP<TinyGsmNRF9151, TINY_GSM_MUX_COUNT>,
                       public TinyGsmSSL<TinyGsmNRF9151, TINY_GSM_MUX_COUNT> {
  friend class TinyGsmModem<TinyGsmNRF9151>;
  friend class TinyGsmGPRS<TinyGsmNRF9151>;
  friend class TinyGsmTCP<TinyGsmNRF9151, TINY_GSM_MUX_COUNT>;
  friend class TinyGsmSSL<TinyGsmNRF9151, TINY_GSM_MUX_COUNT>;

  /*
   * Inner Client
   */
 public:
  class GsmClientNRF9151 : public GsmClient {
    friend class TinyGsmNRF9151;

   public:
    GsmClientNRF9151() {}

    explicit GsmClientNRF9151(TinyGsmNRF9151& modem, uint8_t mux = 0) {
      init(&modem, mux);
    }

    bool init(TinyGsmNRF9151* modem, uint8_t mux = 0) {
      this->at       = modem;
      sock_available = 0;
      prev_check     = 0;
      sock_connected = false;
      got_data       = false;

      if (mux < TINY_GSM_MUX_COUNT) {
        this->mux = mux;
      } else {
        this->mux = (mux % TINY_GSM_MUX_COUNT);
      }
      at->sockets[this->mux] = this;

      return true;
    }

   public:
    virtual int connect(const char* host, uint16_t port, int timeout_s) {
      stop();
      TINY_GSM_YIELD();
      rx.clear();
      sock_connected = at->modemConnect(host, port, mux, false, timeout_s);
      return sock_connected;
    }
    TINY_GSM_CLIENT_CONNECT_OVERRIDES

    void stop(uint32_t maxWaitMs) {
      dumpModemBuffer(maxWaitMs);
      at->closeSocket(mux, maxWaitMs);
      sock_connected = false;
    }
    void stop() override {
      stop(15000L);
    }

    /*
     * Extended API
     */

    String remoteIP() TINY_GSM_ATTR_NOT_IMPLEMENTED;
  };

  /*
   * Inner Secure Client
   */
 public:
  class GsmClientSecureNRF9151 : public GsmClientNRF9151 {
   public:
    GsmClientSecureNRF9151() {}

    explicit GsmClientSecureNRF9151(TinyGsmNRF9151& modem, uint8_t mux = 0)
        : GsmClientNRF9151(modem, mux) {}

    // Selects which stored TLS credential (sec_tag, as provisioned via
    // addCertificate()) this connection presents at connect() time. Nothing
    // is written to the modem.
    bool setCertificate(const String& certificateName) {
      return at->setCertificate(certificateName, mux);
    }

    int connect(const char* host, uint16_t port, int timeout_s) override {
      stop();
      TINY_GSM_YIELD();
      rx.clear();
      sock_connected = at->modemConnect(host, port, mux, true, timeout_s);
      return sock_connected;
    }
    TINY_GSM_CLIENT_CONNECT_OVERRIDES
  };

  /*
   * Constructor
   */
 public:
  explicit TinyGsmNRF9151(Stream& stream) : stream(stream), activeMux(-1) {
    memset(sockets, 0, sizeof(sockets));
    for (uint8_t i = 0; i < TINY_GSM_MUX_COUNT; i++) { socketHandles[i] = -1; }
  }

  /*
   * Basic functions
   */
 protected:
  bool initImpl(const char* pin = nullptr) {
    DBG(GF("### TinyGSM Version:"), TINYGSM_VERSION);
    DBG(GF("### TinyGSM Compiled Module:  TinyGsmClientNRF9151"));

    if (!testAT()) { return false; }

    sendAT(GF("E0"));  // Echo off
    if (waitResponse() != 1) { return false; }

#ifdef TINY_GSM_DEBUG
    sendAT(GF("+CMEE=2"));  // verbose error codes
#else
    sendAT(GF("+CMEE=0"));  // numeric/no error codes
#endif
    waitResponse();

    // Enable verbose unsolicited EPS registration reports (mode 2 also
    // reports cell/location info); this keeps isNetworkConnectedImpl() and
    // the registration helpers fed with fresh state.
    sendAT(GF("+CEREG=2"));
    waitResponse();

    DBG(GF("### Modem:"), getModemName());

    SimStatus ret = getSimStatus();
    // if the sim isn't ready and a pin has been provided, try to unlock it
    if (ret != SIM_READY && pin != nullptr && strlen(pin) > 0) {
      simUnlock(pin);
      return (getSimStatus() == SIM_READY);
    } else {
      // if the sim is ready, or it's locked but no pin has been provided,
      // return true
      return (ret == SIM_READY || ret == SIM_LOCKED);
    }
  }

  bool factoryDefaultImpl() {
    // Not all Nordic SLM builds implement the generic Hayes "&F"/"&W"
    // profile-persistence commands; treat this as unsupported rather than
    // silently pretending it worked.
    return false;
  }

  /*
   * Power functions
   */
 protected:
  bool restartImpl(const char* pin = nullptr) {
    if (!testAT()) { return false; }
    sendAT(GF("#XRESET"));  // reboots the modem
    if (waitResponse(10000L) != 1) { return false; }
    // Give the modem time to fully come back up before re-probing it.
    delay(3000);
    return init(pin);
  }

  bool powerOffImpl() {
    // #XSHUTDOWN performs a full, safe power-off of the modem core, as
    // opposed to #XRESET (reboot back into an operational state).
    sendAT(GF("#XSHUTDOWN"));
    return waitResponse(40000L) == 1;
  }

  // radioOffImpl() is inherited from TinyGsmModem's default implementation,
  // which calls setPhoneFunctionality(0) -- dispatched via CRTP to
  // setPhoneFunctionalityImpl() below, i.e. "AT+CFUN=0".

  bool sleepEnableImpl(bool enable = true) {
    // Maps enable to #XSLEEP mode 1 (idle sleep) and disable to 0.
    sendAT(GF("#XSLEEP="), enable ? 1 : 0);
    return waitResponse(10000L) == 1;
  }

  bool setPhoneFunctionalityImpl(uint8_t fun, bool reset = false) {
    // The SLM passes AT+CFUN through to the modem as documented ("standard
    // 3GPP command"); the `reset` flag has no SLM-documented equivalent, so
    // it is intentionally ignored here rather than guessed at.
    (void)reset;
    sendAT(GF("+CFUN="), fun);
    return waitResponse(10000L) == 1;
  }

  bool setBaudImpl(uint32_t baud) {
    // #XSLMUART configures the UART link between the host MCU and the
    // modem, including its baud rate.
    sendAT(GF("#XSLMUART="), baud);
    return waitResponse() == 1;
  }

  /*
   * Generic network functions
   */
 public:
  NRF9151RegStatus getRegistrationStatus() {
    // LTE-only modem: only EPS (CEREG) registration is meaningful.
    return (NRF9151RegStatus)getRegistrationStatusXREG("CEREG");
  }

 protected:
  bool isNetworkConnectedImpl() {
    NRF9151RegStatus s = getRegistrationStatus();
    return (s == REG_OK_HOME || s == REG_OK_ROAMING);
  }

  // getLocalIPImpl(), getSignalQualityImpl() and waitForNetworkImpl() are
  // inherited from TinyGsmModem's defaults (AT+CGPADDR=1 / AT+CSQ / polling
  // isNetworkConnected()), all of which are standard 3GPP commands that SLM
  // passes through to the modem.

  /*
   * Secure socket layer (SSL) functions
   */
 protected:
  // IMPORTANT: This must only be called *before* attempting to register on
  // the network (i.e. before gprsConnect()/attaching). Writing a TLS
  // credential requires putting the modem in offline mode (AT+CFUN=4); if
  // the modem is already registered, cycling CFUN=4 -> CFUN=1 here will
  // drop that connection. Calling this at the right point in the
  // connection lifecycle is the responsibility of the code that uses this
  // class -- this class does not try to detect or guard against misuse.
  bool addCertificateImpl(const char* certificateName, const char* cert,
                          const uint16_t len) {
    long sec_tag = atol(certificateName);

    // #XCMNG subcommand 1 lists the credentials stored for this sec_tag, so
    // a credential that is already present is not re-written (which would
    // require cycling CFUN again).
    bool alreadyHasCert = false;
    sendAT(GF("#XCMNG=1,"), sec_tag, GF(",0"));
    if (waitResponse(5000L, GF(AT_NL "#XCMNG:")) == 1) {
      streamSkipUntil('\n');  // consume the rest of the listing line
      waitResponse();
      alreadyHasCert = true;
    } else {
      waitResponse();  // consume trailing OK/ERROR of the list attempt
    }

    if (alreadyHasCert) {
      DBG("### Certificate already present for sec_tag", sec_tag,
          "-- not overwriting");
      return true;
    }

    // Credential writes require the modem to be offline first.
    sendAT(GF("+CFUN=4"));
    if (waitResponse(10000L) != 1) { return false; }

    // #XCMNG=<opcode>,<sec_tag>,<type>,<content>
    // opcode 0 = write; type 0 = root CA certificate (1 = client cert, 2 =
    // client private key -- neither is used here, only the CA cert is
    // provisioned for server-authenticated TLS).
    // The certificate body is written directly from the caller-provided
    // buffer/length rather than assuming it is a null-terminated C string.
    stream.print(GF("AT#XCMNG=0,"));
    stream.print(sec_tag);
    stream.print(GF(",0,\""));
    stream.write(reinterpret_cast<const uint8_t*>(cert), len);
    stream.print(GF("\"" AT_NL));
    stream.flush();
    TINY_GSM_YIELD();
    bool success = waitResponse(10000L) == 1;

    // Always try to bring the radio back up regardless of the write
    // result, so we never leave the modem stuck offline.
    sendAT(GF("+CFUN=1"));
    waitResponse(10000L);

    return success;
  }

  bool deleteCertificateImpl(const char* filename) {
    long sec_tag = atol(filename);

    sendAT(GF("+CFUN=4"));
    if (waitResponse(10000L) != 1) { return false; }

    // opcode 3 = delete.
    sendAT(GF("#XCMNG=3,"), sec_tag, GF(",0"));
    bool success = waitResponse(10000L) == 1;

    sendAT(GF("+CFUN=1"));
    waitResponse(10000L);

    return success;
  }

  /*
   * GPRS functions
   */
 protected:
  bool gprsConnectImpl(const char* apn, const char* user = nullptr,
                       const char* pwd = nullptr) {
    // Like u-blox's LTE-M/NB-IoT SARA-R4 modules, the nRF91 series uses a
    // single "external" PDP context set up via the standard 3GPP
    // CGDCONT/CGATT/CGACT commands -- there is no SIMCOM-style separate
    // "application network" activation step here.
    if (user && strlen(user) > 0) {
      sendAT(GF("+CGAUTH=1,0,\""), user, GF("\",\""), pwd, '"');
      waitResponse();
    }

    sendAT(GF("+CGDCONT=1,\"IP\",\""), apn, '"');  // Define PDP context 1
    waitResponse();

    sendAT(GF("+CGATT=1"));  // Attach to the LTE network
    if (waitResponse(360000L) != 1) { return false; }

    sendAT(GF("+CGACT=1,1"));  // Activate PDP context 1
    if (waitResponse(150000L) != 1) { return false; }

    return true;
  }

  bool gprsDisconnectImpl() {
    // Mark all sockets closed first, so async/remote-closed sockets are
    // reflected even if this call itself has trouble.
    for (int mux = 0; mux < TINY_GSM_MUX_COUNT; mux++) {
      GsmClientNRF9151* sock = sockets[mux];
      if (sock && sock->sock_connected) { sock->sock_connected = false; }
    }

    sendAT(GF("+CGACT=0"));  // Deactivate all PDP contexts
    waitResponse(40000L);

    sendAT(GF("+CGATT=0"));  // Detach from the network
    if (waitResponse(360000L) != 1) { return false; }

    return true;
  }

  /*
   * SIM card functions
   */
 protected:
  // This uses "CGSN" (as SARA-R4 also does) rather than the plain "GSN"
  // TinyGsmGPRS default, matching Nordic's documented AT+CGSN support.
  String getIMEIImpl() {
    sendAT(GF("+CGSN"));
    if (waitResponse(GF(AT_NL)) != 1) { return ""; }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  // getIMSIImpl(), getSimCCIDImpl(), getSimStatusImpl(), simUnlockImpl(),
  // isGprsConnectedImpl(), getOperatorImpl() and getProviderImpl() are all
  // inherited from TinyGsmGPRS's defaults, which use standard 3GPP commands
  // (+CIMI, +CCID, +CPIN?, +CGATT?, +COPS?) that SLM passes through to the
  // modem.

  /*
   * Client related (socket) functions
   */
 protected:
  // Makes sure the socket identified by `mux` is the one that subsequent
  // #XSEND/#XRECV operate on. Per the design of this driver, this is called
  // immediately before every single send/receive/connect operation on a
  // given mux -- it deliberately does NOT cache "already selected" state,
  // both for simplicity and because a spontaneous URC (e.g. the peer
  // closing the other socket) could otherwise leave that cache stale.
  bool selectSocket(uint8_t mux) {
    if (mux >= TINY_GSM_MUX_COUNT || socketHandles[mux] < 0) { return false; }
    sendAT(GF("#XSOCKETSELECT="), socketHandles[mux]);
    // The response may optionally echo
    // "#XSOCKETSELECT: <handle>,<type>,<protocol>" ahead of the terminal
    // OK; handle both that and a bare OK/ERROR defensively.
    int8_t res = waitResponse(3000L, GF(AT_NL "#XSOCKETSELECT:"),
                              GFP(GSM_OK), GFP(GSM_ERROR));
    bool   ok  = false;
    if (res == 1) {
      streamSkipUntil('\n');
      ok = waitResponse() == 1;
    } else {
      ok = res == 2;
    }
    if (ok) { activeMux = mux; }
    return ok;
  }

  // Closes and releases the socket (if any) currently assigned to `mux`.
  // Called from GsmClientNRF9151::stop() -- kept here, rather than inlined
  // in the nested client class, so all the AT parsing for it lives
  // alongside the rest of the socket lifecycle code.
  void closeSocket(uint8_t mux, uint32_t maxWaitMs) {
    if (mux >= TINY_GSM_MUX_COUNT || socketHandles[mux] < 0) { return; }
    uint32_t startMillis = millis();
    // #XSOCKET=0 closes whichever socket is currently selected -- this
    // applies the same way whether it was opened with #XSOCKET or with
    // #XSSOCKET.
    if (selectSocket(mux)) {
      sendAT(GF("#XSOCKET=0"));
      uint32_t elapsed = millis() - startMillis;
      if (waitResponse(elapsed > maxWaitMs ? 0 : maxWaitMs - elapsed,
                       GF(AT_NL "#XSOCKET:")) == 1) {
        streamSkipUntil('\n');
      }
      waitResponse(2000L);
    }
    socketHandles[mux] = -1;
    if (activeMux == mux) { activeMux = -1; }
  }

  bool modemConnect(const char* host, uint16_t port, uint8_t mux,
                    bool ssl = false, int timeout_s = 120) {
    uint32_t timeout_ms  = ((uint32_t)timeout_s) * 1000;
    uint32_t startMillis = millis();

    int16_t handle = -1;
    if (ssl) {
      long sec_tag = certificates[mux].length() > 0
          ? atol(certificates[mux].c_str())
          : -1;
      if (sec_tag < 0) {
        DBG("### ERROR: no certificate sec_tag configured for mux", mux,
            "-- call setCertificate() with a sec_tag provisioned via "
            "addCertificate() first");
        return false;
      }
      // #XSSOCKET=<op>,<type>,<role>,<sec_tag>[,<peer_verify>[,<cid>]]
      // op=1 (open IPv4), type=1 (TLS 1.2), role=0 (client)
      sendAT(GF("#XSSOCKET=1,1,0,"), sec_tag);
      if (waitResponse(timeout_ms, GF(AT_NL "#XSSOCKET:")) != 1) {
        return false;
      }
    } else {
      // #XSOCKET=<op>,<type>,<role>
      // op=1 (open IPv4), type=1 (TCP), role=0 (client)
      sendAT(GF("#XSOCKET=1,1,0"));
      if (waitResponse(timeout_ms, GF(AT_NL "#XSOCKET:")) != 1) {
        return false;
      }
    }
    // #XSOCKET:/#XSSOCKET: <handle>,<type>,<protocol>
    handle = streamGetIntBefore(',');
    streamSkipUntil('\n');  // type, protocol
    waitResponse();

    if (handle < 0) { return false; }
    socketHandles[mux] = handle;

    if (!selectSocket(mux)) { return false; }

    // #XCONNECT is shared by plain and native-TLS sockets alike, once the
    // socket has been opened and selected.
    sendAT(GF("#XCONNECT=\""), host, GF("\","), port);
    uint32_t elapsed = millis() - startMillis;
    if (waitResponse(elapsed > timeout_ms ? 0 : timeout_ms - elapsed,
                     GF(AT_NL "#XCONNECT:")) != 1) {
      return false;
    }
    int8_t status = streamGetIntBefore('\n');  // 1 = connected
    waitResponse();
    return status == 1;
  }

  int16_t modemSend(const void* buff, size_t len, uint8_t mux) {
    if (!sockets[mux]) { return 0; }
    if (!selectSocket(mux)) { return 0; }

    size_t chunk = len;
    if (chunk > 1024) { chunk = 1024; }  // #XSEND max payload per Nordic docs

    // #XSEND takes its payload inline, as a quoted string, in the AT command
    // line itself. Embedded quotes and backslashes are escaped below, but
    // raw CR/LF/NUL bytes cannot be represented this way.
    stream.print(GF("AT#XSEND=\""));
    for (size_t i = 0; i < chunk; i++) {
      uint8_t c = reinterpret_cast<const uint8_t*>(buff)[i];
      if (c == '\"' || c == '\\') { stream.write('\\'); }
      stream.write(c);
    }
    stream.print(GF("\"" AT_NL));
    stream.flush();
    TINY_GSM_YIELD();

    if (waitResponse(GF(AT_NL "#XSEND:")) != 1) { return 0; }
    int16_t sent = streamGetIntBefore('\n');
    waitResponse();
    return sent;
  }

  size_t modemRead(size_t size, uint8_t mux) {
    if (!sockets[mux]) { return 0; }
    if (!selectSocket(mux)) { return 0; }

    // #XRECV's only parameter is a receive *timeout* in seconds, not a
    // byte count -- unlike most other TinyGSM modems there is no way to
    // ask for "up to N bytes". We use a short timeout since this is only
    // called once data is already believed to be waiting (via the
    // "#XRECV:" URC, see handleURCs below), and cap what we copy into our
    // local FIFO at `size` -- anything the modem returns beyond that is
    // still read off the wire (to keep the AT parser in sync) but
    // discarded; TINY_GSM_BUFFER_READ_AND_CHECK_SIZE will pick it up again
    // on a subsequent read.
    sendAT(GF("#XRECV=1"));
    if (waitResponse(GF(AT_NL "#XRECV:")) != 1) { return 0; }
    int16_t len = streamGetIntBefore('\n');
    if (len <= 0) {
      waitResponse();
      sockets[mux]->sock_available = 0;
      return 0;
    }

    // #XRECV: <size>\r\n"<data>"\r\nOK
    streamSkipUntil('\"');  // opening quote
    size_t toStore = TinyGsmMin((size_t)len, size);
    for (int i = 0; i < len; i++) {
      if ((size_t)i < toStore) {
        moveCharFromStreamToFifo(mux);
      } else {
        uint32_t startMillis = millis();
        while (!stream.available() &&
               (millis() - startMillis < sockets[mux]->_timeout)) {
          TINY_GSM_YIELD();
        }
        stream.read();
      }
    }
    streamSkipUntil('\"');  // closing quote
    waitResponse();

    sockets[mux]->sock_available = modemGetAvailable(mux);
    return toStore;
  }

  size_t modemGetAvailable(uint8_t mux) {
    if (!sockets[mux]) { return 0; }
    // SLM's #XRECV command has no distinct "peek size" form: any call to it
    // performs (and consumes) a receive, so it cannot be used here without
    // destructively pulling data out of band from modemRead(). Data
    // availability is therefore tracked from the "#XRECV: <size>" URC that
    // the modem pushes when new data arrives (see handleURCs); this call
    // just returns that cached count, while opportunistically re-checking
    // that the socket is still connected so a remote close is still
    // detected even if its own notification was missed.
    size_t result = sockets[mux]->sock_available;
    if (!result) { sockets[mux]->sock_connected = modemGetConnected(mux); }
    return result;
  }

  bool modemGetConnected(uint8_t mux) {
    if (!sockets[mux]) { return false; }
    if (!selectSocket(mux)) { return false; }
    // There is no dedicated "get socket status" query; the read form of
    // #XSOCKET is used to report the state of the currently selected socket.
    sendAT(GF("#XSOCKET?"));
    int8_t res = waitResponse(GF(AT_NL "#XSOCKET:"));
    if (res != 1) {
      // Nothing useful returned; don't change what we already believe.
      return sockets[mux]->sock_connected;
    }
    streamSkipUntil(',');  // handle
    streamSkipUntil(',');  // type
    int8_t state = streamGetIntBefore('\n');  // non-zero -> connected
    waitResponse();
    return state != 0;
  }

  /*
   * Utilities
   */
 public:
  bool handleURCs(String& data) {
    if (data.endsWith(GF("#XRECV:"))) {
      // Unsolicited notification that data has arrived on the currently
      // selected socket. SLM's socket model only allows one socket to be
      // selected at a time, so -- exactly like the explicit send/receive
      // commands -- we attribute this to whichever mux was most recently
      // selected. This is safe here because callers of this library are
      // required to serialize all socket access through an external mutex,
      // so no other select can race with this in practice.
      int16_t len = streamGetIntBefore('\n');
      if (activeMux >= 0 && activeMux < TINY_GSM_MUX_COUNT &&
          sockets[activeMux]) {
        sockets[activeMux]->got_data = true;
        if (len > 0) { sockets[activeMux]->sock_available = len; }
      }
      data = "";
      DBG("### URC Data Received:", len, "on mux", activeMux);
      return true;
    } else if (data.endsWith(GF("#XSOCKET:")) ||
               data.endsWith(GF("#XSSOCKET:")) ||
               data.endsWith(GF("#XCONNECT:"))) {
      // These prefixes are normally synchronous command responses, already
      // consumed directly by modemConnect()/selectSocket() before reaching
      // this generic URC path. The socket commands can however also be
      // reused for spontaneous close/error notifications, whose payload is
      // not parsed here. Since this runs in the middle of parsing another
      // response and must not issue AT commands recursively, the currently
      // active socket is simply marked as disconnected; the next
      // connect()/available() call re-establishes the real state.
      streamSkipUntil('\n');
      if (activeMux >= 0 && activeMux < TINY_GSM_MUX_COUNT &&
          sockets[activeMux]) {
        sockets[activeMux]->sock_connected = false;
      }
      data = "";
      DBG("### Unexpected socket URC on mux", activeMux);
      return true;
    }
    return false;
  }

 public:
  Stream& stream;

 protected:
  GsmClientNRF9151* sockets[TINY_GSM_MUX_COUNT];
  // Modem-assigned socket handle (from #XSOCKET:/#XSSOCKET:) for each mux,
  // or -1 if that mux currently has no open socket.
  int16_t socketHandles[TINY_GSM_MUX_COUNT];
  // Which mux was most recently selected via #XSOCKETSELECT -- used only to
  // attribute unsolicited notifications (see handleURCs) to the right
  // socket, not to skip re-issuing #XSOCKETSELECT before an operation.
  int8_t activeMux;
};

#endif  // SRC_TINYGSMCLIENTNRF9151_H_
