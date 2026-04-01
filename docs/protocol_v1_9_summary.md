# MCON AMOLED Communication Protocol v1.9 (2026-03-31)

This document summarizes the updated protocol based on version 1.9.

## 1. General Packet Structure
| Header | ID Byte | Command (CMD) | Data Len | Payload | Tail |
| :---: | :---: | :---: | :---: | :---: | :---: |
| 0x19 | *Var* | *Var* | *Len* | ... | 0x2F |

- **ID Byte**:
    - `0x4D`: Basic Turn-by-Turn (TBT) / Device Info
    - `0x4E`: Device Information (Version/Brightness/etc)
    - `0x4F`: Firmware Update
    - `0x50`: Image Transfer (**Corrected in v1.9**)

---

## 2. Firmware Update (ID: 0x4F)
| CMD_ID (Hex) | Name | Direction | Length & Payload |
| :--- | :--- | :--- | :--- |
| **0x01** | Update Info | APP -> HUD | 6~13B: Ver(6), Size(4), Sum(2/16) |
| **0x02** | Update Request | HUD -> APP | 3B: Result(1: OK/Fail), SBS(2: Block Size) |
| **0x03** | Update Data | APP -> HUD | 2+SBS Bytes: Seq No(2), Data(SBS) |
| **0x04** | Update Result | HUD -> APP | **DELETED** in v1.8 |
| **0x05** | Update Complete | HUD -> APP | 1B: Error Code (0: Success, 1: Fail) |

---

## 3. Image Transfer (ID: 0x50)
The Image Transfer protocol is now clearly separated from Firmware Update using ID **0x50**.

| CMD_ID (Hex) | Name | Direction | Length & Payload |
| :--- | :--- | :--- | :--- |
| **0x01** | Image Info | APP -> HUD | 6B: Index(1: 1~5), Size(4), Type(1: Intro/Bg) |
| **0x11** | Info Response | HUD -> APP | 2B: SBS (Selected Block Size) |
| **0x02** | Data Transfer | APP -> HUD | 2+SBS Bytes: Seq No(2), Image Data(SBS) |
| **0x03** | Data Ack | HUD -> APP | 3B: Seq No(2), Result Code(1: 0=OK, 1=Retry) |
| **0x04** | Completion Ack | HUD -> APP | 1B: Error Code (0: Success, 1: Fail) |
| **0x05** | Delete Image | APP -> HUD | 1B: Image Index (1~5) |
| **0x06** | Auto Mode | APP -> HUD | 1B: Mode (0: Auto, 1: Manual) |
| **0x07** | Status Request | HUD -> APP | 1B: Current Image Index (1~5) |
| **0x08** | Status Sync | APP -> HUD | 1B: Current Image Index (1~5) |

---

## 4. Key Changes Log
- **v1.7 (03-27)**: Added ID `0x50` (initially `0x4F` override) for image info delivery.
- **v1.8 (03-30)**: Removed `CMD 0x04` (Update Result) from Firmware Update sequence.
- **v1.9 (03-31)**: **Corrected Image Information Response ID from 0x4F to 0x50**. Unified all image commands under ID `0x50`.

> [!IMPORTANT]
> Ensure all Image Transfer responses (0x11, 0x03, 0x04, 0x07) use **ID Byte 0x50** in the header.
