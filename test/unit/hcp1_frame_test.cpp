// The older bus's frames, against the example frames published with the
// protocol description.
//
// Those examples are the closest thing to an outside opinion available here:
// somebody recorded them from a real SupraMatic E3 and wrote down the bytes,
// checksum included. Reproducing them means the checksum, the packing of
// counter and length, and the field order are all right at once - and getting
// any of them wrong would otherwise only show up at a door nobody here owns.
#include <cstdio>
#include <cstring>

#include "hcp1_frame.h"

using namespace esphome::hcpbridge;

namespace {

int failures = 0;

void check(bool ok, const char *what) {
  printf("  %-58s %s\n", what, ok ? "ok" : "FAILED");
  if (!ok)
    failures++;
}

void checkBytes(const uint8_t *got, size_t gotLen, const uint8_t *want, size_t wantLen, const char *what) {
  const bool ok = gotLen == wantLen && memcmp(got, want, wantLen) == 0;
  check(ok, what);
  if (!ok) {
    printf("      built ");
    for (size_t i = 0; i < gotLen; i++)
      printf("%02X ", got[i]);
    printf("\n      want  ");
    for (size_t i = 0; i < wantLen; i++)
      printf("%02X ", want[i]);
    printf("\n");
  }
}

}  // namespace

int main() {
  uint8_t buf[HCP1_MAX_FRAME];

  {
    // "Broadcast status": counter 5, two data bytes, the door reported open.
    const uint8_t want[] = {0x00, 0x52, 0x01, 0x02, 0xD0};
    const uint8_t data[] = {0x01, 0x02};
    const size_t n = hcp1Build(buf, sizeof(buf), HCP1_ADDR_BROADCAST, 5, data, 2);
    checkBytes(buf, n, want, sizeof(want), "a published broadcast is rebuilt byte for byte");
  }

  {
    // "Slave status response": what a UAP1 answers, asking the door to open,
    // with the emergency input healthy.
    const uint8_t want[] = {0x80, 0x63, 0x29, 0x01, 0x10, 0x4B};
    const uint8_t data[] = {HCP1_CMD_STATUS_RESPONSE, HCP1_DO_OPEN, HCP1_S0_HEALTHY};
    const size_t n = hcp1Build(buf, sizeof(buf), HCP1_ADDR_MASTER, 6, data, 3);
    checkBytes(buf, n, want, sizeof(want), "a published answer is rebuilt byte for byte");
  }

  {
    // The same two, read rather than written.
    const uint8_t raw[] = {0x00, 0x52, 0x01, 0x02, 0xD0};
    Hcp1Frame f;
    check(hcp1Parse(raw, sizeof(raw), &f), "the published broadcast parses");
    check(f.address == HCP1_ADDR_BROADCAST && f.counter == 5 && f.length == 2 && f.data[0] == 0x01 && f.data[1] == 0x02,
          "and its fields come out as the description says");
  }

  {
    const uint8_t raw[] = {0x80, 0x63, 0x29, 0x01, 0x10, 0x4B};
    Hcp1Frame f;
    check(hcp1Parse(raw, sizeof(raw), &f), "the published answer parses");
    check(f.counter == 6 && f.length == 3 && f.data[0] == HCP1_CMD_STATUS_RESPONSE,
          "and carries the counter and command it should");
  }

  {
    // One bit flipped anywhere has to be refused; that is what the checksum is
    // there for, and on this bus it is only eight bits wide.
    uint8_t raw[] = {0x80, 0x63, 0x29, 0x01, 0x10, 0x4B};
    bool allRefused = true;
    for (size_t byte = 0; byte < sizeof(raw); byte++)
      for (int bit = 0; bit < 8; bit++) {
        raw[byte] ^= (uint8_t) (1 << bit);
        Hcp1Frame f;
        if (hcp1Parse(raw, sizeof(raw), &f))
          allRefused = false;
        raw[byte] ^= (uint8_t) (1 << bit);
      }
    check(allRefused, "every single-bit change to a good frame is refused");
  }

  {
    Hcp1Frame f;
    const uint8_t truncated[] = {0x80, 0x63, 0x29};
    check(!hcp1Parse(truncated, sizeof(truncated), &f), "a frame shorter than it claims is refused");
    const uint8_t trailing[] = {0x00, 0x52, 0x01, 0x02, 0xD0, 0x00};
    check(!hcp1Parse(trailing, sizeof(trailing), &f), "a frame longer than it claims is refused");
    check(!hcp1Parse(truncated, 2, &f), "and so is one too short to hold a checksum");
  }

  {
    check(hcp1NextCounter(6) == 7, "the answer counts one past what arrived");
    check(hcp1NextCounter(15) == 0, "and wraps within its four bits");
  }

  {
    // Longer than the four bits the length field has.
    uint8_t big[32] = {0};
    check(hcp1Build(buf, sizeof(buf), HCP1_ADDR_MASTER, 1, big, 16) == 0,
          "a payload the length field cannot describe is refused");
    check(hcp1Build(buf, 4, HCP1_ADDR_MASTER, 1, big, 8) == 0, "and so is one that would not fit the buffer");
  }

  printf(failures ? "\nFAILED\n" : "\nall HCP1 frame checks passed\n");
  return failures ? 1 : 0;
}
