
import os
import base64

def generate():
    esptool_js_path = r"c:\vscode\movition_ws\update\esptool.js"
    output_html = r"c:\vscode\movition_ws\update\update_tool.html"
    
    with open(esptool_js_path, "rb") as f:
        js_data = f.read()
    
    b64_js = base64.b64encode(js_data).decode('utf-8')
    
    html_template = """<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Movition 펌웨어 업데이트 도구 (Standalone)</title>
    <style>
        :root {
            --primary: #4f46e5;
            --primary-hover: #4338ca;
            --bg: #f8fafc;
            --card-bg: #ffffff;
            --text: #1e293b;
            --text-muted: #64748b;
            --success: #22c55e;
            --error: #ef4444;
            --border: #e2e8f0;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            background-color: var(--bg);
            color: var(--text);
            margin: 0;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
        }

        .container {
            width: 100%;
            max-width: 500px;
            padding: 2.5rem;
            background: var(--card-bg);
            border-radius: 1.5rem;
            box-shadow: 0 10px 25px -5px rgba(0, 0, 0, 0.1);
            margin: 1rem;
        }

        h1 {
            font-size: 1.5rem;
            font-weight: 700;
            margin-bottom: 0.5rem;
            text-align: center;
            color: var(--primary);
        }

        .subtitle {
            text-align: center;
            color: var(--text-muted);
            font-size: 0.9rem;
            margin-bottom: 2rem;
        }

        .step-card {
            border: 1px solid var(--border);
            border-radius: 1rem;
            padding: 1.25rem;
            margin-bottom: 1.25rem;
            background-color: #fafafa;
        }

        .step-title {
            font-weight: 600;
            margin-bottom: 1rem;
            display: flex;
            align-items: center;
            gap: 0.75rem;
        }

        .step-number {
            background: var(--primary);
            color: white;
            width: 24px;
            height: 24px;
            border-radius: 50%;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 0.8rem;
        }

        button {
            width: 100%;
            padding: 0.875rem;
            border: none;
            border-radius: 0.75rem;
            background-color: var(--primary);
            color: white;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s;
            font-size: 1rem;
        }

        button:hover:not(:disabled) {
            background-color: var(--primary-hover);
            transform: translateY(-1px);
        }

        button:disabled {
            background-color: var(--border);
            color: var(--text-muted);
            cursor: not-allowed;
        }

        .hidden { display: none; }

        #log {
            margin-top: 1.5rem;
            padding: 1rem;
            background: #1e293b;
            color: #f1f5f9;
            border-radius: 0.75rem;
            font-family: 'Consolas', 'Monaco', monospace;
            font-size: 0.8rem;
            height: 180px;
            overflow-y: auto;
            white-space: pre-wrap;
            line-height: 1.6;
        }

        .progress-container {
            margin-top: 1.25rem;
            height: 10px;
            background: var(--border);
            border-radius: 5px;
            overflow: hidden;
        }

        #progressBar {
            height: 100%;
            background: linear-gradient(90deg, #4f46e5, #818cf8);
            width: 0%;
            transition: width 0.3s;
        }

        .status-msg {
            font-size: 0.85rem;
            margin-top: 0.5rem;
            text-align: center;
            font-weight: 500;
        }

        .file-info {
            background: #f1f5f9;
            padding: 0.5rem;
            border-radius: 0.5rem;
            margin-bottom: 0.75rem;
            font-size: 0.85rem;
            text-align: center;
            border: 1px dashed var(--border);
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Movition Updater</h1>
        <p class="subtitle">모비전 펌웨어 및 데이터를 안전하게 전송합니다.</p>

        <div id="step1" class="step-card">
            <div class="step-title"><span class="step-number">1</span> 업데이트 파일 준비</div>
            <div id="fileInfo" class="file-info">update.bin 파일을 선택해 주세요.</div>
            <input type="file" id="fileInput" accept=".bin" class="hidden">
            <button id="selectFileBtn">파일 선택</button>
        </div>

        <div id="step2" class="step-card">
            <div class="step-title"><span class="step-number">2</span> 업데이트 실행</div>
            <button id="connectBtn" disabled>업데이트 시작</button>
            <div class="progress-container hidden" id="progressWrapper">
                <div id="progressBar"></div>
            </div>
            <div id="status" class="status-msg">파일을 먼저 선택해 주세요.</div>
        </div>

        <div id="log">시스템 준비 중...</div>
    </div>

    <script type="module">
        // --- EMBEDDED ENGINE DATA ---
        const b64Data = "{{BASE64_PLACEHOLDER}}";
        
        let targetLib = null;
        let engineReady = false;

        const logArea = document.getElementById('log');
        const statusMsg = document.getElementById('status');
        const selectFileBtn = document.getElementById('selectFileBtn');
        const connectBtn = document.getElementById('connectBtn');
        const fileInput = document.getElementById('fileInput');
        const fileInfo = document.getElementById('fileInfo');
        const progressBar = document.getElementById('progressBar');
        const progressWrapper = document.getElementById('progressWrapper');

        const log = (msg) => {
            const time = new Date().toLocaleTimeString();
            logArea.innerText += `[${time}] ${msg}\\n`;
            logArea.scrollTop = logArea.scrollHeight;
        };

        const ui8ToBstr = (ui8) => {
            let str = "";
            const chunk = 8192;
            for (let i = 0; i < ui8.length; i += chunk) {
                str += String.fromCharCode.apply(null, ui8.subarray(i, i + chunk));
            }
            return str;
        };

        const loadEngine = async () => {
            try {
                log("엔진 로딩 중...");
                const blob = new Blob([Uint8Array.from(atob(b64Data), c => c.charCodeAt(0))], { type: 'application/javascript' });
                const url = URL.createObjectURL(blob);
                const module = await import(url);
                URL.revokeObjectURL(url);
                log("엔진 로딩 완료.");
                return module;
            } catch (err) {
                log("❌ 엔진 로딩 실패: " + err.message);
                throw err;
            }
        };

        selectFileBtn.onclick = () => fileInput.click();

        fileInput.onchange = () => {
            if (fileInput.files.length > 0) {
                const file = fileInput.files[0];
                fileInfo.innerText = `📦 ${file.name} (${(file.size / 1024 / 1024).toFixed(2)} MB)`;
                if (engineReady) {
                    connectBtn.disabled = false;
                    statusMsg.innerText = "연결할 준비가 되었습니다.";
                }
            }
        };

        connectBtn.onclick = async () => {
            if (!fileInput.files.length) return;
            
            try {
                log("사용자 포트 요청 중...");
                const port = await navigator.serial.requestPort();
                
                const transport = new targetLib.Transport(port);
                const esploader = new targetLib.ESPLoader({
                    transport: transport,
                    baudrate: 460800,
                    romBaudrate: 115200,
                    terminal: {
                        clean: () => { logArea.innerText = ""; },
                        writeLine: (data) => log(data),
                        write: (data) => log(data)
                    }
                });

                statusMsg.innerText = "장치와 통신 시도 중...";
                await esploader.main();
                
                log("업데이트 패키지 분석 중...");
                const fileReader = new FileReader();
                
                fileReader.onload = async (e) => {
                    const fullBuffer = e.target.result;
                    const view = new DataView(fullBuffer);
                    
                    if (view.getUint32(0, true) !== 0x44475055) {
                        log("❌ 오류: 유효한 패키지가 아닙니다.");
                        statusMsg.innerText = "잘못된 파일 형식";
                        return;
                    }

                    const appSize = view.getUint32(4, true);
                    const storageSize = view.getUint32(8, true);
                    log(`패키지 인식: 앱(${appSize} bytes), 데이터(${storageSize} bytes)`);

                    const fileArray = [];
                    let pointer = 16;

                    if (appSize > 0) {
                        const slice = fullBuffer.slice(pointer, pointer + appSize);
                        fileArray.push({
                            data: ui8ToBstr(new Uint8Array(slice)),
                            address: 0x20000
                        });
                        pointer += appSize;
                        log("- 앱 바이너리 변환 완료");
                    }

                    if (storageSize > 0) {
                        const slice = fullBuffer.slice(pointer, pointer + storageSize);
                        fileArray.push({
                            data: ui8ToBstr(new Uint8Array(slice)),
                            address: 0x3B2000
                        });
                        log("- 데이터 바이너리 변환 완료");
                    }

                    progressWrapper.classList.remove('hidden');
                    statusMsg.innerText = "업데이트 진행 중...";

                    try {
                        await esploader.writeFlash({
                            fileArray: fileArray,
                            flashSize: "keep",
                            flashMode: "keep",
                            flashFreq: "keep",
                            eraseAll: false,
                            compress: true,
                            reportProgress: (fileIndex, written, total) => {
                                const percent = Math.floor((written / total) * 100);
                                progressBar.style.width = percent + "%";
                                statusMsg.innerText = `전송 중... [${fileIndex + 1}/${fileArray.length}] ${percent}%`;
                            }
                        });

                        log("✅ 성공: 모든 데이터가 전송되었습니다!");
                        statusMsg.innerText = "완료! 장치가 재부팅됩니다.";
                        await esploader.after("hard_reset");
                    } catch (err) {
                        log("❌ 전송 오류: " + err.message);
                        statusMsg.innerText = "전송 실패";
                    }
                };
                
                fileReader.readAsArrayBuffer(fileInput.files[0]);

            } catch (err) {
                log("❌ 연결 오류: " + err.message);
                statusMsg.innerText = "연결 실패";
            }
        };

        (async () => {
            try {
                targetLib = await loadEngine();
                engineReady = true;
                logArea.innerText = "시스템 준비 완료.\\n";
                if (fileInput.files.length > 0) {
                    connectBtn.disabled = false;
                    statusMsg.innerText = "준비 완료!";
                } else {
                    statusMsg.innerText = "업데이트 파일을 선택해 주세요.";
                }
            } catch (e) {
                statusMsg.innerText = "시스템 초기화 실패";
            }
        })();
    </script>
</body>
</html>
"""
    
    final_content = html_template.replace("{{BASE64_PLACEHOLDER}}", b64_js)
    
    with open(output_html, "w", encoding="utf-8") as f:
        f.write(final_content)
    print("Success: Standalone HTML regenerated with string data fix.")

if __name__ == "__main__":
    generate()
