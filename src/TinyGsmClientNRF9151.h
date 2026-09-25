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
 *    it creates (e.g. via an external mutex) so that concurrent operations
 *    on different muxes are never raced by another thread/task. No locking
 *    is done inside this class.
 */

#ifndef SRC_TINYGSMCLIENTNRF9151_H_
#define SRC_TINYGSMCLIENTNRF9151_H_
// #pragma message("TinyGSM:  TinyGsmClientNRF9151")

// #define TINY_GSM_DEBUG Serial

// Two simultaneous sockets are supported, e.g. one MQTT-over-TLS connection
// and one HTTP-over-TLS connection.
#define TINY_GSM_MUX_COUNT 2
// Size of the receive request issued when polling a socket for data.
#ifndef TINY_GSM_RX_BUFFER
#define TINY_GSM_RX_BUFFER 1024
#endif
// Maximum number of bytes requested by a single "#XRECV".
#ifndef TINY_GSM_NRF9151_RECV_SEGMENT
#define TINY_GSM_NRF9151_RECV_SEGMENT 2048
#endif
// How long the modem may block in zsock_recv() waiting for socket data (the
// <timeout> field of "#XRECV"). zsock_recv() returns as soon as any data is
// available, so this is only the upper bound on an idle wait.
#ifndef TINY_GSM_NRF9151_RECV_TIMEOUT_S
#define TINY_GSM_NRF9151_RECV_TIMEOUT_S 1
#endif
// The host-side wait for that command's response MUST exceed the modem-side
// timeout above. Otherwise waitResponse() abandons an "#XRECV" that is still
// in flight, the modem's late reply desynchronises the AT stream, and every
// response slower than the host-side wait costs a wasted poll cycle.
#ifndef TINY_GSM_NRF9151_RECV_WAIT_MS
#define TINY_GSM_NRF9151_RECV_WAIT_MS \
  ((TINY_GSM_NRF9151_RECV_TIMEOUT_S * 1000L) + 1000L)
#endif
// How long after the last outbound send (or last byte received) a socket is
// considered to be "actively exchanging", and so polled for incoming data as
// fast as the modem will answer.
#ifndef TINY_GSM_NRF9151_RX_BURST_MS
#define TINY_GSM_NRF9151_RX_BURST_MS 3000L
#endif
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
      sock_connected  = false;
      got_data        = false;
      _txLen          = 0;
      _lastExchangeMs = 0;
      _awaitingReply  = false;

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
      _txLen         = 0;  // drop anything buffered but never sent -- the
                           // socket it was destined for is going away.
      _awaitingReply = false;
      dumpModemBuffer(maxWaitMs);
      at->closeSocket(mux, maxWaitMs);
      sock_connected = false;
    }
    void stop() override {
      stop(15000L);
    }

    size_t write(const uint8_t* buf, size_t size) override {
      size_t written = 0;
      while (written < size) {
        size_t space = sizeof(_txBuf) - _txLen;
        if (space == 0) {
          if (!flushTx()) { break; }
          space = sizeof(_txBuf);
        }
        size_t chunk = TinyGsmMin(space, size - written);
        memcpy(_txBuf + _txLen, buf + written, chunk);
        _txLen += chunk;
        written += chunk;
      }
      return written;
    }
    size_t write(uint8_t c) override {
      return write(&c, 1);
    }

    size_t availableLocal() { return (size_t)rx.size(); }

    int available() override {
      flushTx();
      if (rx.size()) {
        _lastExchangeMs = millis();
        return (int)rx.size() + sock_available;
      }
      // Poll the modem immediately only while a reply is outstanding.
      if (_awaitingReply &&
          millis() - _lastExchangeMs < TINY_GSM_NRF9151_RX_BURST_MS) {
        got_data   = true;
        prev_check = millis();
      }
      int avail = GsmClient::available();
      if (avail > 0) {
        _lastExchangeMs = millis();
        _awaitingReply  = false;
      }
      return avail;
    }

    int read(uint8_t* buf, size_t size) override {
      flushTx();
      if (rx.size()) {
        size_t chunk = TinyGsmMin(size, (size_t)rx.size());
        rx.get(buf, chunk);
        _lastExchangeMs = millis();
        _awaitingReply  = false;
        return (int)chunk;
      }
      int got = GsmClient::read(buf, size);
      if (got > 0) {
        _lastExchangeMs = millis();
        _awaitingReply  = false;
      }
      return got;
    }

    int read() override {
      uint8_t c;
      if (read(&c, 1) == 1) { return (int)c; }
      return -1;
    }

    void flush() override {
      flushTx();
      GsmClient::flush();
    }

    uint8_t connected() override {
      flushTx();
      if (rx.size()) { return true; }
      return sock_connected;
    }

    /*
     * Extended API
     */

    String remoteIP() TINY_GSM_ATTR_NOT_IMPLEMENTED;

   protected:
    // Sends any buffered outgoing bytes as a single #XSEND and clears the
    // buffer.
    bool flushTx() {
      if (_txLen == 0) { return true; }
      // Having just sent something, a reply is expected imminently
      _lastExchangeMs = millis();
      _awaitingReply  = true;
      int16_t sent = at->modemSend(_txBuf, _txLen, mux);
      bool    ok   = (sent >= 0 && (size_t)sent == _txLen);
      _txLen       = 0;
      return ok;
    }

    // Sized to modemSend()'s own #XSEND payload cap (Nordic's documented
    // maximum), so a full buffer always flushes as exactly one #XSEND.
    uint8_t  _txBuf[1024];
    size_t   _txLen = 0;
    // Last time this socket sent or received anything; gates burst polling.
    uint32_t _lastExchangeMs = 0;
    // True between sending something and receiving anything back, i.e. only
    // while a blocking receive can actually be expected to find data. See
    // available().
    bool _awaitingReply = false;
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
    // #XSLEEP modes: 1 terminates the SLM service and the LTE connection,
    // 2 is an idle sleep in which only the UART is disabled. Enabling sleep
    // uses mode 1.
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
    sendAT(GF("+IPR="), baud);
    return waitResponse() == 1;
  }

  /*
   * Network system mode
   */
 public:
  bool getNetworkSystemMode(bool& n, int16_t& stat) {
    // Read:
    // %XSYSTEMMODE: <lte_m_support>, <nb_iot_support>,
    //               <gnss_support>, <lte_preference>

    sendAT(GF("%XSYSTEMMODE?"));

    if (waitResponse(
            GF(AT_NL "%XSYSTEMMODE:")) != 1) {
        return false;
    }

    const int16_t lteMEnabled = streamGetIntBefore(',');
    const int16_t nbIotEnabled = streamGetIntBefore(',');
    const int16_t gnssEnabled = streamGetIntBefore(',');

    stat = streamGetIntBefore('\n');

    if (waitResponse() != 1) {
        return false;
    }

    // Preference 1 explicitly prefers LTE-M.
    // Preference 3 also prefers LTE-M when network priorities are equal.
    n = (stat == 1 || stat == 3);

    (void)gnssEnabled;

    // Reports success only if both LTE-M and NB-IoT are enabled.
    return lteMEnabled == 1 && nbIotEnabled == 1;
  }

  bool setNetworkSystemMode(bool n = true) {
    // Enables LTE-M only (NB-IoT and GNSS are left disabled).
    // %XSYSTEMMODE must be changed while the modem is inactive.
    const int8_t preference = n ? 1 : 0;

    if (!setPhoneFunctionality(0)) {
        return false;
    }

    sendAT(
        GF("%XSYSTEMMODE="),
        1, GF(","),       // LTE-M enabled
        0, GF(","),       // NB-IoT enabled
        0, GF(","),       // GNSS disabled
        preference        // LTE-M preferred or automatic
    );

    if (waitResponse() != 1) {
        return false;
    }

    return setPhoneFunctionality(1);
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
  // credential requires putting the modem in minimum-functionality mode
  // (AT+CFUN=0 or 4); if the modem is already registered, cycling CFUN=0->1
  // here will drop that connection. Calling this at the right point in the
  // connection lifecycle is the responsibility of the code that uses this
  // class -- this class does not try to detect or guard against misuse.
  bool addCertificateImpl(const char* certificateName, const char* cert,
                          const uint16_t len) {
    long sec_tag = atol(certificateName);

    // %CMNG is the modem's native credential-storage command (passed through
    // by SLM as-is). Subcommand 1 lists the credentials stored for this
    // sec_tag, so a credential that is already present is not re-written
    // (which would require cycling CFUN again).
    bool alreadyHasCert = false;
    sendAT(GF("%CMNG=1,"), sec_tag, GF(",0"));
    if (waitResponse(5000L, GF(AT_NL "%CMNG:")) == 1) {
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

    // Credential writes require minimum-functionality mode or flight mode.
    sendAT(GF("+CFUN=0"));
    if (waitResponse(10000L) != 1) { return false; }

    // The certificate is sent one PEM line at a time, as a single quoted
    // string spanning several lines.
    stream.print(GF("AT%CMNG=0,"));
    stream.print(sec_tag);
    stream.print(GF(",0,\""));
    const char* p   = cert;
    const char* end = cert + len;
    if (end > p && *(end - 1) == '\n') { end--; }  // trailing '\n' is not its own line
    while (p < end) {
      const char* nl =
          static_cast<const char*>(memchr(p, '\n', end - p));
      const char* lineEnd = nl ? nl : end;
      stream.write(reinterpret_cast<const uint8_t*>(p), lineEnd - p);
      if (nl) {
        stream.print(AT_NL);
      } else {
        stream.print(GF("\"" AT_NL));  // last line: close the quoted string
      }
      stream.flush();
      delay(150);  // give the SLM's line parser time to keep up
      p = nl ? nl + 1 : end;
    }
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

    sendAT(GF("+CFUN=0"));
    if (waitResponse(10000L) != 1) { return false; }

    // opcode 3 = delete.
    sendAT(GF("%CMNG=3,"), sec_tag, GF(",0"));
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
  // This uses "CGSN" rather than the plain "GSN" used by the TinyGsmGPRS
  // default, matching Nordic's documented AT+CGSN support.
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
  // Closes and releases the socket (if any) currently assigned to `mux`.
  // Called from GsmClientNRF9151::stop() -- kept here, rather than inlined
  // in the nested client class, so all the AT parsing for it lives
  // alongside the rest of the socket lifecycle code.
  void closeSocket(uint8_t mux, uint32_t maxWaitMs) {
    if (mux >= TINY_GSM_MUX_COUNT || socketHandles[mux] < 0) { return; }
    uint32_t startMillis = millis();
    sendAT(GF("#XCLOSE="), socketHandles[mux]);
    if (waitResponse(maxWaitMs, GF(AT_NL "#XCLOSE:")) == 1) {
      streamSkipUntil('\n');
    }
    uint32_t elapsed = millis() - startMillis;
    waitResponse(elapsed > maxWaitMs ? 0 : maxWaitMs - elapsed);
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
      sendAT(GF("#XSSOCKET=1,1,0,"), sec_tag, GF(",2,0"));
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
    activeMux = mux;

    if (ssl) {
      // Set the TLS peer name after the socket-open response has been fully
      // consumed, before starting the connection.
      sendAT(GF("#XSSOCKETOPT="), handle, GF(",1,2,\""), host, GF("\""));
      if (waitResponse(timeout_ms) != 1) {
        closeSocket(mux, 15000L);
        return false;
      }
    }

    sendAT(GF("#XCONNECT="), handle, GF(",\""), host, GF("\","), port);
    uint32_t elapsed = millis() - startMillis;
    if (waitResponse(elapsed > timeout_ms ? 0 : timeout_ms - elapsed,
                     GF(AT_NL "#XCONNECT:")) != 1) {
      // The socket was opened but never connected -- close it so our
      // bookkeeping doesn't keep believing this mux owns a live handle
      // (the modem itself may drop it independently on a failed connect).
      closeSocket(mux, 15000L);
      return false;
    }

    // #XCONNECT: <handle>,<status> (status 1 = connected)
    // The handle field must be consumed before the status; reading the whole
    // remainder as one integer would return the handle instead.
    const int16_t connHandle = streamGetIntBefore(',');
    const int8_t  status     = streamGetIntBefore('\n');
    waitResponse();
    if (connHandle != handle || status != 1) {
      closeSocket(mux, 15000L);
      return false;
    }
    return true;
  }

 protected:
  // A payload containing a NUL cannot go out as a mode-0 quoted string:
  // the SLM's AT parser treats the 0x00 as end-of-string before any
  // quote/escape handling runs, so the command line is truncated and
  // rejected outright.
  static bool payloadNeedsDataMode(const void* buff, size_t len) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(buff);
    for (size_t i = 0; i < len; i++) {
      if (p[i] == 0x00) { return true; }
    }
    return false;
  }

  // Mode 2 ("data mode"): the byte count is declared up front and the payload
  // is then streamed raw, so none of it is parsed as AT syntax. Completion
  // arrives asynchronously as "#XDATAMODE: <status>" (0 = success) instead of
  // the synchronous "#XSEND:" that mode 0 returns, and no "+++" terminator is
  // needed because <data_len> was supplied.
  int16_t modemSendDataMode(const void* buff, size_t chunk, int16_t handle) {
    // Terminated with a bare CR, deliberately not sendAT(): the modem is
    // built with CONFIG_SM_CR_TERMINATION, so it dispatches the command on
    // the CR and switches into data mode synchronously, before the next byte
    // off the UART is parsed.
    stream.print(GF("AT#XSEND="));
    stream.print(handle);
    stream.print(GF(",2,0,"));
    stream.print((uint32_t)chunk);
    stream.print('\r');
    stream.flush();
    TINY_GSM_YIELD();
    if (waitResponse(3000L) != 1) { return 0; }

    const uint8_t* p       = reinterpret_cast<const uint8_t*>(buff);
    size_t         written = 0;
    uint32_t       lastProgressMs = millis();
    while (written < chunk) {
      size_t n = stream.write(p + written, chunk - written);
      if (n > 0) {
        written += n;
        lastProgressMs = millis();
      } else if (millis() - lastProgressMs > 10000L) {
        break;
      }
      TINY_GSM_YIELD();
    }
    stream.flush();
    TINY_GSM_YIELD();

    if (written != chunk) {
      // The modem is still waiting for the remainder, so the AT channel is
      // unusable until it times out on its own. Report the failure; the
      // caller's socket-level error handling has to take it from here.
      DBG("### #XSEND data mode short write:", written, "of", chunk);
      return 0;
    }

    if (waitResponse(10000L, GF(AT_NL "#XDATAMODE:")) != 1) { return 0; }
    const int16_t status = streamGetIntBefore('\n');
    return status == 0 ? (int16_t)chunk : 0;
  }

  int16_t modemSend(const void* buff, size_t len, uint8_t mux) {
    if (!sockets[mux] || socketHandles[mux] < 0) { return 0; }

    size_t chunk = len;
    if (chunk > 1024) { chunk = 1024; }  // #XSEND max payload per Nordic docs

    const int16_t handle = socketHandles[mux];
    if (payloadNeedsDataMode(buff, chunk)) {
      return modemSendDataMode(buff, chunk, handle);
    }

    // AT#XSEND=<handle>,0,0,"payload"
    // Response is:
    // #XSEND: <handle>,<flags>,<bytes_sent>
    // The payload remains inline as a quoted string, but the socket handle
    // and the 0,0 flags must precede it.
    stream.print(GF("AT#XSEND="));
    stream.print(handle);
    stream.print(GF(",0,0,\""));
    for (size_t i = 0; i < chunk; i++) {
      uint8_t c = reinterpret_cast<const uint8_t*>(buff)[i];
      if (c == '\"' || c == '\\') { stream.write('\\'); }
      stream.write(c);
    }
    stream.print(GF("\"" AT_NL));
    stream.flush();
    TINY_GSM_YIELD();

    if (waitResponse(GF(AT_NL "#XSEND:")) != 1) { return 0; }
    streamSkipUntil(',');  // handle
    streamSkipUntil(',');  // flags
    int16_t sent = streamGetIntBefore('\n');
    waitResponse();
    return sent;
  }

  // #XRECV is handle-scoped on this SLM build. The final argument is
  // the maximum number of bytes to return; the response contains raw bytes,
  // not a quoted string.
  size_t modemRead(size_t size, uint8_t mux) {
    if (!sockets[mux] || socketHandles[mux] < 0) { return 0; }

    const int16_t handle = socketHandles[mux];
    const size_t requestSize =
        TinyGsmMin(size, static_cast<size_t>(TINY_GSM_NRF9151_RECV_SEGMENT));
    // #XRECV=<handle>,<flags>,<reserved>,<timeout>,<max_size>
    // flags 0, reserved 0. Note the explicit GF(",") separators: sendAT()
    // concatenates its arguments with no delimiter of its own.
    sendAT(GF("#XRECV="), handle, GF(",0,0,"),
           TINY_GSM_NRF9151_RECV_TIMEOUT_S, GF(","), requestSize);
    // The host-side wait deliberately exceeds the modem-side timeout above
    // (see TINY_GSM_NRF9151_RECV_WAIT_MS).
    if (waitResponse(TINY_GSM_NRF9151_RECV_WAIT_MS, GF(AT_NL "#XRECV:")) != 1) {
      return 0;
    }

    const int16_t responseHandle = streamGetIntBefore(',');
    streamSkipUntil(',');  // flags
    const int16_t len = streamGetIntBefore('\n');
    if (responseHandle != handle || len <= 0) {
      waitResponse();
      sockets[mux]->sock_available = 0;
      return 0;
    }

    // #XRECV: <handle>,<flags>,<size>\r\n<data>\r\nOK
    size_t toStore = TinyGsmMin((size_t)len, requestSize);
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
    waitResponse();

    sockets[mux]->sock_available = 0;
    return toStore;
  }

  size_t modemGetAvailable(uint8_t mux) {
    if (!sockets[mux] || socketHandles[mux] < 0) { return 0; }

    // This SLM build reports readable data through its internal socket poll
    // state, but does not emit a #XRECV URC over the AT interface. TinyGSM's
    // available() path reaches here before read(), so issue the receive here
    // when no URC has already supplied a byte count. modemRead() places the
    // raw response bytes directly into the client RX FIFO.
    if (!sockets[mux]->sock_available) {
      modemRead(TINY_GSM_RX_BUFFER, mux);
    }
    return sockets[mux]->sock_available;
  }

  bool modemGetConnected(uint8_t mux) {
    if (!sockets[mux] || socketHandles[mux] < 0) { return false; }

    const int16_t expectedHandle = socketHandles[mux];
    // The documented read-form query is:
    //   AT#XSOCKET?
    //   #XSOCKET: <handle>,<family>,<role>,<type>,<cid>
    // This modem exposes a global socket table, not a per-mux one. Treating
    // the mere presence of any valid handle as "our socket is still up" is
    // wrong when a different socket (or a stale security-tagged one) is open
    // at the same time.
    sendAT(GF("#XSOCKET?"));
    int8_t res = waitResponse(GF(AT_NL "#XSOCKET:"));
    if (res == 0) {
      return sockets[mux]->sock_connected;
    }
    if (res != 1) {
      // The command completed (OK/ERROR) but with no "#XSOCKET:" listing
      // at all -- the modem's socket table is empty, so our handle is
      // definitely gone. Clear our own bookkeeping too, so callers (e.g.
      // modemGetAvailable()) stop re-polling a handle that no longer
      // exists anywhere instead of doing so forever.
      socketHandles[mux] = -1;
      if (activeMux == mux) { activeMux = -1; }
      return false;
    }
    int16_t handle = streamGetIntBefore(',');
    streamSkipUntil('\n');
    waitResponse();
    return handle == expectedHandle;
  }

  /*
   * Utilities
   */
 public:
  // Rough RSRP guide: > -90 excellent, -90..-105 good,
  // -105..-115 marginal (expect retransmissions and low throughput),
  // < -115 poor.
  int16_t getRsrpDbm() const {
    return (_cesqRsrp == 255 || _cesqRsrp < 0) ? 0 : (int16_t)(_cesqRsrp - 140);
  }
  int16_t getRsrqDb() const {
    return (_cesqRsrq == 255 || _cesqRsrq < 0) ? 0 : (int16_t)((_cesqRsrq / 2) - 19);
  }

  // Actively refreshes the RSRP/RSRQ values above via the standard 3GPP
  // AT+CESQ query.
  bool refreshSignalQuality() {
    sendAT(GF("+CESQ"));
    if (waitResponse(GF(AT_NL "+CESQ:")) != 1) { return false; }
    streamSkipUntil(',');  // rxlev
    streamSkipUntil(',');  // ber
    streamSkipUntil(',');  // rscp
    streamSkipUntil(',');  // ecno
    // +CESQ: <rxlev>,<ber>,<rscp>,<ecno>,<rsrq>,<rsrp>
    _cesqRsrq = streamGetIntBefore(',');
    _cesqRsrp = streamGetIntBefore('\n');
    waitResponse();
    return true;
  }

  bool handleURCs(String& data) {
    if (data.endsWith(GF("%CESQ:"))) {
      // "%CESQ: <rsrp>,<rsrp_threshold>,<rsrq>,<rsrq_threshold>" -- the two
      // signal values are recorded for getRsrpDbm()/getRsrqDb(); the
      // threshold indices aren't needed.
      _cesqRsrp = streamGetIntBefore(',');
      streamSkipUntil(',');  // rsrp threshold index
      _cesqRsrq = streamGetIntBefore(',');
      streamSkipUntil('\n');
      data = "";
      return true;
    } else if (data.endsWith(GF("#XRECV:"))) {
      // Unsolicited notification that data has arrived. This URC's payload
      // is not confirmed to carry a handle of its own on this firmware, so
      // -- like the explicit #XSEND/#XRECV commands -- it is attributed to
      // whichever mux was most recently connected/operated on.
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
      // Normally consumed directly by modemConnect()/closeSocket(); an
      // unexpected arrival is treated as the active socket having been
      // closed. The next connect()/available() call re-establishes the real
      // state.
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
  // Modem-assigned socket handle (from #XSOCKET:/#XSSOCKET:) for each mux
  int16_t socketHandles[TINY_GSM_MUX_COUNT];
  // Which mux was most recently connected/operated on -- used only to
  // attribute unsolicited notifications (see handleURCs) to the corresponding
  // socket
  int8_t activeMux;
  // Signal quality as raw reported indices (from the "%CESQ" URC or
  // refreshSignalQuality()); see getRsrpDbm()/getRsrqDb().
  int16_t _cesqRsrp = -1;
  int16_t _cesqRsrq = -1;
};

#endif  // SRC_TINYGSMCLIENTNRF9151_H_
