// ===============================================
// 1. GLOBAL STATE & CONSTANTS
// ===============================================

// BLE Configuration (使用您的 UUID)
const SERVICE_UUID = '12a59900-17cc-11ec-9621-0242ac130002'; 
const CHARACTERISTIC_UUID_TX = '12a5a148-17cc-11ec-9621-0242ac130002'; // Notify Characteristic (Data Reception)
const CHARACTERISTIC_UUID_RX = '12a59e0a-17cc-11ec-9621-0242ac130002'; // Write Characteristic (Command Sending)
const CHARACTERISTIC_UUID = CHARACTERISTIC_UUID_TX; // 用于简化数据流逻辑

const MAX_DATA_POINTS = 150; // Visible points on the scrolling chart
const CONFIDENCE_UPDATE_MS = 500; // 0.5s update for confidence

let bleDevice = null;
let bleGattServer = null;
let bleCharacteristicTx = null;
let bleCharacteristicRx = null;

let isConnected = false;
let isPredictionRunning = false;
let confidenceInterval = null;
let dataBuffer = []; // 存储 RAW 和 Normalized 数据
let chartInstances = {};
let selectedMode = 'walk'; 
let packetStats = { received: 0, lastTime: 0, rateHz: 0, rssi: 0 };


// --- MODEL CONSTANTS (Hardcoded from logistic_model.json) ---
const LOGISTIC_MODEL = {
  "classes": [ "jump", "stair", "walk" ],
  "coef": [
    [ 1.30516, -0.29828, -0.78946, 0.03881, 0.27593, 0.86539, 0.05485, -0.11355, -0.39463, 0.34009, 0.92249, -1.66316, 1.03957 ], // jump
    [ -0.40664, 0.17238, -0.02003, -0.12403, -1.37618, -0.75987, -0.24862, 0.64635, -1.94101, 0.78173, -0.70459, 0.59144, 0.79399 ], // stair
    [ -0.89852, 0.12589, 0.80949, 0.08521, 1.10024, -0.10551, 0.19376, -0.53279, 2.33564, -1.12183, -0.21789, 1.07171, -1.83357 ]  // walk
  ],
  "intercept": [ -1.15582, -0.23144, 1.38726 ],
  "feature_names": [
    "mmg_rms", "mmg_mean", "mmg_std", "mmg_pp", 
    "emg_rms", "emg_mean", "emg_std", "emg_pp", 
    "emg_zcr", "acc_mean", "acc_std", "gyro_mean", "gyro_std"
  ],
  "scaler_mean": [ 0.41865, 0.00125, 0.18152, 0.77386, 450.20720, 430.27070, 67.97362, 268.60837, 0.00310, 17105.2745, 623.98492, 2542.3183, 914.80163 ],
  "scaler_scale": [ 0.33856, 0.44003, 0.25162, 1.07672, 735.60043, 726.42543, 162.31860, 620.54868, 0.02604, 652.62641, 1542.2837, 2999.2736, 1578.8560 ]
};


// Normalization Constants (for CHARTING ONLY - using typical ranges)
const NORM_CONSTANTS = {
    ACCEL_MIN: -20000, ACCEL_MAX: 20000,
    MMG_MIN: 500, MMG_MAX: 1000,
    EMG_MIN: 0, EMG_MAX: 1000,
};

// Prediction Ranges (Updated to reflect model classes, 'run' is excluded in the model)
const PREDICTION_RANGES = {
    walk: { text: "WALK", min: 0.88, max: 0.96 },
    run: { text: "RUN", min: 0.83, max: 0.92 }, // Kept for UI logic/initial selected mode
    stair: { text: "STAIR", min: 0.86, max: 0.93 },
    jump: { text: "JUMP", min: 0.84, max: 0.91 }
};

// ===============================================
// 2. DOM ELEMENTS
// ===============================================

const deviceSelect = document.getElementById('device-select');
const connectButton = document.getElementById('connect-button');
const connectionStatusEl = document.getElementById('connection-status');
const modeSelectButton = document.getElementById('mode-select-button'); 
const modeFloatingMenu = document.getElementById('mode-floating-menu'); 
const startPredictionButton = document.getElementById('start-prediction-button');
const predictedActionEl = document.getElementById('predicted-action');
const confidenceValueEl = document.getElementById('confidence-value');
const systemStatusEl = document.getElementById('system-status');


// ===============================================
// 3. DATA UTILITIES & NORMALIZATION
// ===============================================

function normalize(value, min, max) {
    if (max - min === 0) return 0;
    const normalized = (value - min) / (max - min);
    return Math.max(0, Math.min(1, normalized)); 
}

function normalizeAndEnhance(value, min, max, contrast = 1.8) {
    const norm = normalize(value, min, max); 
    const enhanced = (norm - 0.5) * contrast + 0.5;
    return Math.max(0, Math.min(1, enhanced));
}

/**
 * Parses RAW sensor data from BLE and calculates Normalized data for charting.
 */
function parseBLEPacket(jsonString) {
    let raw = {};

    try {
        raw = JSON.parse(jsonString);
    } catch (e) {
        // Fallback for CSV format (simplified based on the expected 10 components)
        const values = jsonString.split(',');
        if (values.length >= 10) {
             raw = {
                ax: parseFloat(values[0]), ay: parseFloat(values[1]), az: parseFloat(values[2]),
                m1: parseFloat(values[3]), m2: parseFloat(values[4]), m3: parseFloat(values[5]),
                emg: parseFloat(values[6]), gx: parseFloat(values[7]), gy: parseFloat(values[8]), gz: parseFloat(values[9])
            };
        } else {
            return null; // Data is insufficient or malformed
        }
    }

    // Assumptions for RAW data presence (must match ESP32 output):
    if (isNaN(raw.ax) || isNaN(raw.m1) || isNaN(raw.emg) || isNaN(raw.gx)) return null;


    // 1. Calculate NORM data for charting (based on constant ranges)
    const accx = normalizeAndEnhance(raw.ax, NORM_CONSTANTS.ACCEL_MIN, NORM_CONSTANTS.ACCEL_MAX);
    const accy = normalizeAndEnhance(raw.ay, NORM_CONSTANTS.ACCEL_MIN, NORM_CONSTANTS.ACCEL_MAX);
    const accz = normalizeAndEnhance(raw.az, NORM_CONSTANTS.ACCEL_MIN, NORM_CONSTANTS.ACCEL_MAX);
    const mmg1 = normalize(raw.m1, NORM_CONSTANTS.MMG_MIN, NORM_CONSTANTS.MMG_MAX);
    const mmg2 = normalize(raw.m2, NORM_CONSTANTS.MMG_MIN, NORM_CONSTANTS.MMG_MAX);
    const mmg3 = normalize(raw.m3, NORM_CONSTANTS.MMG_MIN, NORM_CONSTANTS.MMG_MAX);
    const emg = normalize(raw.emg, NORM_CONSTANTS.EMG_MIN, NORM_CONSTANTS.EMG_MAX);

    return {
        // RAW data (for ML feature calculation)
        raw_accx: raw.ax, raw_accy: raw.ay, raw_accz: raw.az,
        raw_mmg1: raw.m1, raw_mmg2: raw.m2, raw_mmg3: raw.m3,
        raw_emg: raw.emg, raw_gx: raw.gx, raw_gy: raw.gy, raw_gz: raw.gz,
        // Normalized data (for Charting)
        accx, accy, accz, mmg1, mmg2, mmg3, emg
    };
}

function getSmoothedEMG(dataArray, currentEMG, windowSize = 10) {
    const lastN = dataArray.slice(-windowSize + 1).map(d => d.emg); 
    lastN.push(currentEMG);
    if (lastN.length === 0) return currentEMG;
    return lastN.reduce((a, b) => a + b, 0) / lastN.length;
}


// ===============================================
// 4. ML FEATURE EXTRACTION & INFERENCE
// ===============================================

/**
 * Calculates a single feature (e.g., RMS, Mean, STD) for a given sensor axis array.
 */
function calculateSingleFeature(data, type) {
    if (data.length === 0) return 0;
    
    const sum = data.reduce((a, b) => a + b, 0);
    const mean = sum / data.length;

    switch (type) {
        case 'mean':
            return mean;
        case 'std':
            const squaredDiffSum = data.map(x => Math.pow(x - mean, 2)).reduce((a, b) => a + b, 0);
            return Math.sqrt(squaredDiffSum / data.length);
        case 'rms': // Root Mean Square
            const squaredSum = data.map(x => Math.pow(x, 2)).reduce((a, b) => a + b, 0);
            return Math.sqrt(squaredSum / data.length);
        case 'pp': // Peak-to-Peak
            return Math.max(...data) - Math.min(...data);
        case 'zcr': // Zero-Crossing Rate
            let zcrCount = 0;
            for (let i = 1; i < data.length; i++) {
                if ((data[i - 1] > 0 && data[i] < 0) || (data[i - 1] < 0 && data[i] > 0)) {
                    zcrCount++;
                }
            }
            return zcrCount / data.length;
        default:
            return 0;
    }
}

/**
 * Calculates the 13 features required by the Logistic Regression model.
 */
function calculateFeaturesML(buffer) {
    if (buffer.length < MAX_DATA_POINTS / 2) {
        return LOGISTIC_MODEL.feature_names.map(() => 0);
    }
    
    // Extract raw sensor arrays
    const mmg_total = buffer.map(d => d.raw_mmg1 + d.raw_mmg2 + d.raw_mmg3);
    const emg = buffer.map(d => d.raw_emg);
    const acc_mag = buffer.map(d => Math.sqrt(Math.pow(d.raw_accx, 2) + Math.pow(d.raw_accy, 2) + Math.pow(d.raw_accz, 2)));
    const gyro_mag = buffer.map(d => Math.sqrt(Math.pow(d.raw_gx, 2) + Math.pow(d.raw_gy, 2) + Math.pow(d.raw_gz, 2)));


    // 1. mmg_rms, 2. mmg_mean, 3. mmg_std, 4. mmg_pp
    const mmg_features = [
        calculateSingleFeature(mmg_total, 'rms'),
        calculateSingleFeature(mmg_total, 'mean'),
        calculateSingleFeature(mmg_total, 'std'),
        calculateSingleFeature(mmg_total, 'pp')
    ];

    // 5. emg_rms, 6. emg_mean, 7. emg_std, 8. emg_pp, 9. emg_zcr
    const emg_features = [
        calculateSingleFeature(emg, 'rms'),
        calculateSingleFeature(emg, 'mean'),
        calculateSingleFeature(emg, 'std'),
        calculateSingleFeature(emg, 'pp'),
        calculateSingleFeature(emg, 'zcr')
    ];

    // 10. acc_mean, 11. acc_std
    const acc_features = [
        calculateSingleFeature(acc_mag, 'mean'),
        calculateSingleFeature(acc_mag, 'std')
    ];
    
    // 12. gyro_mean, 13. gyro_std
    const gyro_features = [
        calculateSingleFeature(gyro_mag, 'mean'),
        calculateSingleFeature(gyro_mag, 'std')
    ];
    
    return [
        ...mmg_features,
        ...emg_features,
        ...acc_features,
        ...gyro_features
    ];
}


/**
 * Runs the integrated Logistic Regression model (Scaling -> Softmax).
 * **[关键修改]**：此函数现在计算并返回最高概率 (pk) 作为真实置信度。
 * @param {Array<number>} features - The 13 calculated features.
 * @returns {{label: string, confidence: number}} The predicted class and its probability.
 */
function runMLInference(features) {
    // 检查数据完整性
    if (features.some(f => f === 0) || features.length !== 13) {
        return { label: 'IDLE', confidence: 0 }; 
    }
    
    const { coef, intercept, classes, scaler_mean, scaler_scale } = LOGISTIC_MODEL;
    const num_classes = classes.length;
    const num_features = features.length;
    
    // 1. Scaling (StandardScaler equivalent: (x - mean) / scale)
    const scaledFeatures = features.map((x, i) => (x - scaler_mean[i]) / scaler_scale[i]);
    
    // 2. Linear Model: Calculate Logits (z_k)
    const scores = [];
    for (let k = 0; k < num_classes; k++) {
        let score = intercept[k];
        for (let j = 0; j < num_features; j++) {
            score += coef[k][j] * scaledFeatures[j];
        }
        scores.push(score);
    }
    
    // 3. Softmax (Calculate Probabilities p_k = exp(z_k) / sum(exp(z_j)))
    const expScores = scores.map(s => Math.exp(s));
    const sumExpScores = expScores.reduce((a, b) => a + b, 0);
    const probabilities = expScores.map(es => es / sumExpScores);
    
    // 4. Find the max probability (pk) and corresponding class
    let maxProb = 0;
    let maxIndex = -1;

    for (let i = 0; i < probabilities.length; i++) {
        if (probabilities[i] > maxProb) {
            maxProb = probabilities[i];
            maxIndex = i;
        }
    }
    
    // Set a minimum confidence threshold
    const predictionLabel = (maxProb > 0.6) ? classes[maxIndex].toUpperCase() : 'IDLE';

    return { label: predictionLabel, confidence: maxProb };
}

/**
 * Runs the prediction (Kept the old name for easy integration with existing call sites.)
 */
function runInference(features) {
    return runMLInference(features);
}


// ===============================================
// 5. CHART UTILITIES
// ===============================================
// ... (Chart functions remain unchanged) ...
function createChart(id, title, yAxisMin, yAxisMax, datasetsConfig) {
    const ctx = document.getElementById(id).getContext('2d');
    
    if (chartInstances[id]) {
        chartInstances[id].destroy();
    }

    const datasets = datasetsConfig.map(config => ({
        label: config.label,
        borderColor: config.color,
        data: Array(MAX_DATA_POINTS).fill(null),
        pointRadius: 0,
        tension: 0.1,
        borderWidth: 2
    }));

    chartInstances[id] = new Chart(ctx, {
        type: 'line',
        data: {
            labels: Array(MAX_DATA_POINTS).fill(''),
            datasets: datasets
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                x: {
                    type: 'category',
                    title: { display: true, text: 'Time (ms)', color: 'var(--text-color-medium)' },
                    grid: { display: false }
                },
                y: {
                    title: { display: true, text: title, color: 'var(--text-color-medium)' },
                    min: yAxisMin,
                    max: yAxisMax,
                    ticks: {
                        color: 'var(--text-color-medium)',
                        maxTicksLimit: 5
                    }
                }
            },
            plugins: {
                legend: { position: 'bottom', labels: { boxWidth: 10, color: 'var(--text-color-dark)' } },
                title: { display: false }
            },
            animation: false,
            elements: { line: { borderWidth: 2 } }
        }
    });
}

function initAllCharts() {
    const normTitle = 'Normalized Value (0-1)';

    // 1. Accelerometer chart (Enhanced Normalized 0-1)
    createChart('accel-chart', normTitle, 0, 1, [
        { label: 'Acc X', color: '#FF9500' }, 
        { label: 'Acc Y', color: '#34AADC' }, 
        { label: 'Acc Z', color: '#34C759' }  
    ]);
    
    // 2. MMG chart (Normalized 0-1)
    createChart('mmg-chart', normTitle, 0, 1, [
        { label: 'MMG1', color: '#AF52DE' }, // Purple
        { label: 'MMG2', color: '#FF9500' }, // Orange
        { label: 'MMG3', color: '#00C7E5' }  // Teal
    ]);
    
    // 3. EMG chart (Normalized 0-1)
    createChart('emg-chart', normTitle, 0, 1, [
        { label: 'EMG', color: '#5856D6' } // Indigo
    ]);
}

/**
 * Updates all charts with the latest data point from the buffer.
 */
function updateAllCharts(dataPoint) {
    const chartsToUpdate = [
        { id: 'accel-chart', values: [dataPoint.accx, dataPoint.accy, dataPoint.accz] },
        { id: 'mmg-chart', values: [dataPoint.mmg1, dataPoint.mmg2, dataPoint.mmg3] },
        { id: 'emg-chart', values: [dataPoint.emg_smoothed] }
    ];
    
    const label = new Date().toLocaleTimeString('en-US');

    chartsToUpdate.forEach(({ id, values }) => {
        const chart = chartInstances[id];
        if (!chart) return;
        
        // Rolling update: Shift and push label/values
        chart.data.labels.shift();
        chart.data.labels.push(label);

        values.forEach((val, i) => {
            chart.data.datasets[i].data.shift();
            chart.data.datasets[i].data.push(val);
        });

        chart.update('none'); 
    });
}


// ===============================================
// 6. BLUETOOTH & REAL-TIME HANDLERS
// ===============================================

let statInterval = null; 

/**
 * Sends a command to the ESP32 via the Write characteristic.
 */
async function sendBLECommand(command) {
    if (!bleCharacteristicRx || !isConnected) return;
    
    try {
        const encoder = new TextEncoder();
        const data = encoder.encode(command);
        await bleCharacteristicRx.writeValue(data);
        console.log(`Sent command: ${command}`);
    } catch (error) {
        console.error("Error sending BLE command:", error);
    }
}

/**
 * Handles incoming BLE notification data.
 */
function handleCharacteristicValueChange(event) {
    const value = event.target.value;
    const rawData = new TextDecoder().decode(value);
    
    packetStats.received++;
    
    const dataPoint = parseBLEPacket(rawData);
    if (!dataPoint) return;
    
    // Calculate Smoothed EMG (using normalized data for chart)
    dataPoint.emg_smoothed = getSmoothedEMG(dataBuffer, dataPoint.emg);

    // Maintain rolling buffer size
    dataBuffer.push(dataPoint);
    if (dataBuffer.length > MAX_DATA_POINTS) {
        dataBuffer.shift();
    }
    
    // 1. Update Charts (ALWAYS run when connected)
    updateAllCharts(dataPoint);
    
    // 2. Run ML Inference (ONLY if prediction is active)
    if (isPredictionRunning) {
        const features = calculateFeaturesML(dataBuffer);
        const { label, confidence } = runInference(features);

        // Update action label immediately
        predictedActionEl.textContent = label;
        
        // Confidence update interval will read the current prediction label and use the real confidence value
        confidenceValueEl.textContent = confidence.toFixed(2);
    }
}

/**
 * Handle BLE device disconnection.
 */
function handleDisconnect(event) {
    if (event && event.type === 'gattserverdisconnected') {
        console.warn("BLE Disconnected:", event.reason);
    }
    
    isConnected = false;
    isPredictionRunning = false;
    
    // UI Reset
    connectButton.textContent = 'Connect';
    connectButton.classList.remove('success');
    connectButton.disabled = false;
    deviceSelect.disabled = false;
    
    connectionStatusEl.textContent = 'Disconnected';
    connectionStatusEl.classList.remove('success');
    connectionStatusEl.classList.add('error');
    
    startPredictionButton.textContent = 'Start Prediction';
    startPredictionButton.classList.remove('stop-state');
    startPredictionButton.disabled = true;

    stopConfidenceUpdates();
    clearInterval(statInterval);
    
    updatePredictionUI("IDLE", "--");
    systemStatusEl.textContent = 'Disconnected. Ready to connect device.';
}


/**
 * Main BLE Connect/Disconnect Handler.
 */
async function handleConnectBLE() {
    if (!navigator.bluetooth) {
        connectionStatusEl.textContent = 'Error: Web Bluetooth not supported.';
        connectionStatusEl.classList.add('error');
        return;
    }

    if (isConnected) {
        if (bleGattServer) {
            await bleGattServer.disconnect();
        }
        handleDisconnect();
        return;
    }
    
    // --- Connection Start UI ---
    connectButton.disabled = true;
    deviceSelect.disabled = true;
    connectionStatusEl.innerHTML = '<span class="spinner"></span>Connecting...';
    connectionStatusEl.classList.remove('error', 'success');
    updatePredictionUI("INITIALIZING...", "--"); 
    systemStatusEl.textContent = 'Scanning for devices...';

    try {
        // 使用 acceptAllDevices: true 替换 filters
        bleDevice = await navigator.bluetooth.requestDevice({
            acceptAllDevices: true,
            optionalServices: [SERVICE_UUID] 
        });
        
        // Connect GATT
        bleGattServer = await bleDevice.gatt.connect();
        
        // Add Disconnect Listener
        bleDevice.addEventListener('gattserverdisconnected', handleDisconnect);

        // Get Service and Characteristics
        const service = await bleGattServer.getPrimaryService(SERVICE_UUID);
        
        // 1. TX Characteristic (NOTIFY - Receive Data)
        bleCharacteristicTx = await service.getCharacteristic(CHARACTERISTIC_UUID_TX);
        bleCharacteristicTx.addEventListener('characteristicvaluechanged', handleCharacteristicValueChange);
        await bleCharacteristicTx.startNotifications();
        
        // 2. RX Characteristic (WRITE - Send Command)
        bleCharacteristicRx = await service.getCharacteristic(CHARACTERISTIC_UUID_RX);

        isConnected = true;
        
        // Start Stat/Signal tracking
        startStatUpdates();
        packetStats.rssi = -65; // Mock initial RSSI
        
        // Send initial 'start' command to the ESP32 
        await sendBLECommand("start"); 
        
        // --- Connection Success UI ---
        connectButton.textContent = 'Connected ✓';
        connectButton.classList.remove('error');
        connectButton.classList.add('success');
        connectButton.disabled = false;
        
        connectionStatusEl.textContent = 'Connected Successfully';
        connectionStatusEl.classList.remove('error');
        connectionStatusEl.classList.add('success');
        startPredictionButton.disabled = false;
        
        updatePredictionUI("IDLE", "--"); 
        systemStatusEl.textContent = `Device connected. Selected Action: ${PREDICTION_RANGES[selectedMode].text}`;
        
    } catch (error) {
        console.error('BLE Connection Failed:', error);
        handleDisconnect(error); 
        connectionStatusEl.textContent = 'Connection Failed';
        connectionStatusEl.classList.add('error');
    }
}

// ===============================================
// 7. PREDICTION & STATUS LOGIC
// ===============================================

/**
 * Centralized function to update prediction card UI (Text only).
 */
function updatePredictionUI(actionName, confidenceValue) {
    predictedActionEl.textContent = actionName;
    confidenceValueEl.textContent = confidenceValue;
}


function stopConfidenceUpdates() {
    if (confidenceInterval) {
        clearInterval(confidenceInterval);
        confidenceInterval = null;
    }
}

// Confidence update interval handler
function confidenceStream() {
    if (!isPredictionRunning) return;
    
    // 1. 重新计算特征并运行 ML 推理以获取当前最新的真实置信度
    const features = calculateFeaturesML(dataBuffer);
    const { label, confidence } = runInference(features);

    // 2. 更新 UI
    predictedActionEl.textContent = label;
    confidenceValueEl.textContent = confidence.toFixed(2);

    // 3. 更新系统状态
    const statusText = `Prediction: ${label} (${confidence.toFixed(2)}) | Signal: ${getSignalQualityText()} (${packetStats.rateHz.toFixed(1)} Hz)`;
    systemStatusEl.textContent = statusText;
}

/**
 * Toggles the prediction and inference process.
 */
function handleStartPrediction() {
    if (!isConnected) return;

    if (isPredictionRunning) {
        // STOPPED state
        stopConfidenceUpdates(); 
        isPredictionRunning = false;
        startPredictionButton.textContent = 'Start Prediction';
        startPredictionButton.classList.remove('stop-state');
        
        systemStatusEl.textContent = `Streaming data. Prediction stopped. Signal: ${getSignalQualityText()} (${packetStats.rateHz.toFixed(1)} Hz)`;
        updatePredictionUI("IDLE", "--");
        
        // Send 'stop' command to ESP32
        sendBLECommand("stop");

    } else {
        // STARTING state
        isPredictionRunning = true;
        
        updatePredictionUI("INITIALIZING...", "--"); 
        
        startPredictionButton.textContent = 'Stop Prediction';
        startPredictionButton.classList.add('stop-state');

        // Start dynamic confidence updates (Runs inference every 500ms)
        confidenceInterval = setInterval(confidenceStream, CONFIDENCE_UPDATE_MS);
        confidenceStream(); // Run once immediately
        
        // Send 'start' command to ESP32
        sendBLECommand("start");
    }
}


// ===============================================
// 8. STATS & SIGNAL QUALITY LOGIC
// ===============================================

function startStatUpdates() {
    clearInterval(statInterval); // Clear old interval
    packetStats.received = 0;
    packetStats.lastTime = performance.now();
    packetStats.rateHz = 0;
    
    // Update stats and signal quality every 1 second
    statInterval = setInterval(updateStats, 1000); 
}

function getSignalQualityText() {
    const rssi = packetStats.rssi;
    if (rssi > -60) return '🟢 Strong';
    if (rssi > -80) return '🟡 Medium';
    return '🔴 Weak';
}

function updateStats() {
    if (!isConnected) return;
    
    const now = performance.now();
    const elapsedTime = (now - packetStats.lastTime) / 1000;
    
    // 1. Calculate Packet Rate
    packetStats.rateHz = packetStats.received / elapsedTime;
    
    // 2. Simple Mock RSSI adjustment based on packet rate 
    if (packetStats.rateHz < 10 && packetStats.rssi > -95) {
        packetStats.rssi -= 2;
    } else if (packetStats.rssi < -65) {
        packetStats.rssi += 1;
    }
    
    // 3. Reset counters for next interval
    packetStats.received = 0;
    packetStats.lastTime = now;

    // 4. Update System Status (only if prediction is OFF)
    if (!isPredictionRunning) {
        const statusText = `Streaming data. Signal: ${getSignalQualityText()} (${packetStats.rateHz.toFixed(1)} Hz)`;
        systemStatusEl.textContent = statusText;
    }
}


// ===============================================
// 9. UI LOGIC (Mode Selector)
// ===============================================

function handleModeSelection(e) {
    if (e.target.classList.contains('mode-option')) {
        const newMode = e.target.getAttribute('data-mode');
        selectedMode = newMode;
        
        modeFloatingMenu.classList.add('hidden');
        
        if (isConnected) {
            const actionText = PREDICTION_RANGES[selectedMode].text;
            
            if (isPredictionRunning) {
                // Restart confidence stream to immediately update prediction/confidence based on new mode's context (though model is mode-agnostic)
                stopConfidenceUpdates();
                confidenceInterval = setInterval(confidenceStream, CONFIDENCE_UPDATE_MS);
                confidenceStream(); 
                systemStatusEl.textContent = `Action changed to ${actionText}. Inference rules updated.`;
            } else {
                systemStatusEl.textContent = `Device connected. Selected Action: ${actionText}`;
            }
        }
    }
}

function toggleModeMenu() {
    modeFloatingMenu.classList.toggle('hidden');
}

// ===============================================
// 10. INITIALIZATION
// ===============================================

function init() {
    // Initial UI setup
    startPredictionButton.disabled = true; 
    systemStatusEl.textContent = 'Ready to connect device.'; 
    updatePredictionUI("IDLE", "--"); 
    
    // Event Listeners
    connectButton.addEventListener('click', handleConnectBLE); 
    startPredictionButton.addEventListener('click', handleStartPrediction);
    
    // Mode Selector Listeners
    modeSelectButton.addEventListener('click', toggleModeMenu);
    modeFloatingMenu.addEventListener('click', handleModeSelection);
    document.addEventListener('click', (e) => {
        if (!modeSelectButton.contains(e.target) && !modeFloatingMenu.contains(e.target)) {
            modeFloatingMenu.classList.add('hidden');
        }
    });

    // Initial chart setup (empty state)
    initAllCharts();
    
    // Initialize mode button with NO content (empty string)
    modeSelectButton.textContent = ''; 
}

// Run the main initialization function
init();