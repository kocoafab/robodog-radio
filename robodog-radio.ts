/**
* Control Robodog
*/
let legPos: number[][] = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [1, 0, 0, 1], [0, 1, 1, 0], [1, 1, 0, 0], [0, 0, 1, 1], [1, 1, 1, 1]];
//% groups='["Connection", "Motion", "LED", "Sound", "Sensors", "AI Settings", "AI Data"]'
//% block="Robodog" weight=80 color=#376db5 icon="\uf1b0"
namespace robodog {
    let isInit = 0;
    let battery = 0;
    let tof = 0;
    let yaw = 0;
    let controlYaw = 0;
    let lastRawYaw = 0;
    let hasYawSample = false;
    let roll = 0;
    let pitch = 0;
    let buttonPressed = false;
    let cameraAlive = false;
    let aiData = pins.createBuffer(10);
    let timerCnt = 0;
    let txData = pins.createBuffer(48);
    let extTxData = pins.createBuffer(10);
    let txTemp = pins.createBuffer(48);
    let xmitBuf = pins.createBuffer(17);
    let ledData = pins.createBuffer(34);
    export let counter = 0;
    let radioInit = false;
    let radioTxIndex = 0;
    let isExtPacketEnabled = false;
    let rotationWaitRelative = false;
    let rotationWaitAbsolute = true;
    let rotationToleranceDeg = 5;
    let rotationTimeoutMs = 3000;
    let rotationPollMs = 20;

    function checksum(buf: Buffer): number {
        let sum = 0;
        if (buf[4] > buf.length)
            return -1;
        for (let i = 6; i < buf[4]; i++) {
            sum += buf[i];
        }
        return sum & 0xFF;
    }

    function checksum2(buf: Buffer): number {
        let sum = 0;
        for (let i = 2; i < buf.length; i++) {
            sum += buf[i];
        }
        return sum & 0xFF;
    }

    function wrapYawDelta(currentYaw: number, previousYaw: number): number {
        let delta = currentYaw - previousYaw
        if (delta > 32767)
            delta -= 65536
        else if (delta < -32768)
            delta += 65536
        return delta
    }

    function rebaseControlYawIfIdle(): void {
        if (txData[21] != 0)
            return
        if (controlYaw > 720)
            controlYaw -= 360
        else if (controlYaw < -720)
            controlYaw += 360
    }

    function initializePackets(): void {
        txData[0] = 0x26; txData[1] = 0xA8; txData[2] = 0x14; txData[3] = 0x81; txData[4] = 48;
        extTxData[0] = 0x26; extTxData[1] = 0xA8; extTxData[2] = 0x14; extTxData[3] = 0x87;
        extTxData[4] = 0x09; extTxData[6] = 0x01;
    }

    function ensurePacketsInitialized(): void {
        if (isInit != 0)
            return;
        initializePackets();
        isInit = 1;
    }

    function updateAlternatingLedPayload(): void {
        if ((txData[14] & 0xC0) != 0xC0)
            return;

        if ((timerCnt % 2) == 0) {
            txData[14] = ledData[0];
            for (let p = 0; p < 16; p++)
                txData[24 + p] = ledData[2 + p];
        }
        else {
            txData[14] = ledData[1];
            for (let p = 0; p < 16; p++)
                txData[24 + p] = ledData[18 + p];
        }
    }

    function clearHeadLedPayload(): void {
        for (let n = 0; n < 16; n++) {
            txData[24 + n] = 0;
            ledData[n + 2] = 0;
        }
    }

    function prepareHeadLedPayload(nextMode: number): void {
        // ledData[0] keeps the last staged head LED mode while txData[14] may be alternating.
        if ((ledData[0] & 0xBF) != nextMode)
            clearHeadLedPayload();
        else {
            for (let n = 0; n < 16; n++)
                txData[24 + n] = ledData[n + 2];
        }
        txData[14] = (txData[14] & 0xC0) | nextMode;
    }

    function stageHeadLedPayload(): void {
        ledData[0] = txData[14];
        ledData[1] = ledData[1] | 0x80;
        for (let n = 0; n < 16; n++)
            ledData[n + 2] = txData[n + 24];
    }

    function serviceRadioTx(): void {
        if (!radioInit)
            return;

        if (isExtPacketEnabled) {
            extTxData[5] = checksum(extTxData);
            radio.sendBuffer(extTxData);
            return;
        }

        if (radioTxIndex == 0) {
            updateAlternatingLedPayload();
            rebaseControlYawIfIdle();
            txData[5] = checksum(txData);
            txTemp.write(0, txData);
        }

        xmitBuf.setNumber(NumberFormat.UInt8LE, 0, radioTxIndex)
        xmitBuf.write(1, txTemp.slice(radioTxIndex * 16, 16))
        radio.sendBuffer(xmitBuf)
        timerCnt += 1;
        radioTxIndex = radioTxIndex >= 2 ? 0 : radioTxIndex + 1;
    }

    loops.everyInterval(30, function () {
        ensurePacketsInitialized();
        if (radioInit || isExtPacketEnabled)
            serviceRadioTx();
    });

    function updateYawState(nextYaw: number): void {
        yaw = nextYaw;
        if (!hasYawSample)
            controlYaw = yaw
        else
            controlYaw += wrapYawDelta(yaw, lastRawYaw);
        lastRawYaw = yaw;
        hasYawSample = true;
    }

    function updateRadioSensorState(receivedBuffer: Buffer): void {
        battery = receivedBuffer[2]
        tof = receivedBuffer[3]
        roll = deflib.toSigned8(receivedBuffer[4])
        pitch = deflib.toSigned8(receivedBuffer[5])
        updateYawState(deflib.toSigned16((receivedBuffer[7] << 8) | receivedBuffer[6]))
        buttonPressed = (receivedBuffer[8] & 0x01) == 1
        cameraAlive = receivedBuffer[9] != 0
    }

    radio.onReceivedBuffer(function (receivedBuffer) {
        if (checksum2(receivedBuffer) != receivedBuffer[1])
            return;

        if ((receivedBuffer[0] & 0x0F) == 0x07) {
            isExtPacketEnabled = false;
            return;
        }

        if (receivedBuffer.length < 10)
            return;

        updateRadioSensorState(receivedBuffer);
        for (let p = 0; p < 10 && (10 + p) < receivedBuffer.length; p++)
            aiData[p] = receivedBuffer[10 + p];
    })

    function checkModeChange(initValue: number, mode: number): void {
        if (txData[15] != mode) {
            for (let i = 16; i < 24; i++)
                txData[i] = initValue;
            txData[15] = mode;
        }
    }

    function getHeadLedOffsets(what: deflib.HeadLedSide): number[] {
        switch (what) {
            case deflib.HeadLedSide.Left:
                return [0];
            case deflib.HeadLedSide.Right:
                return [8];
            default:
                return [0, 8];
        }
    }

    function sanitizeRotationAngle(value: number): number {
        value = Math.abs(Math.round(value))
        return deflib.constrain(value, 0, 360)
    }

    function normalizeUserRangeToInt(value: number, fromMin: number, fromMax: number, toMin: number, toMax: number): number {
        value = deflib.constrain(value, fromMin, fromMax)
        return Math.round(toMin + ((value - fromMin) * (toMax - toMin)) / (fromMax - fromMin))
    }

    function normalizeHeading360(value: number): number {
        value = Math.round(value) % 360
        if (value < 0)
            value += 360
        return value
    }

    function getShortestHeadingDelta(currentHeading: number, targetHeading: number): number {
        let delta = normalizeHeading360(targetHeading - currentHeading)
        if (delta > 180)
            delta -= 360
        return delta
    }

    function clearRotationCommand(): void {
        txData[21] = 0
        txData[22] = 0
        txData[23] = 0
    }

    function writeRotationTarget(target: number, velocity: number): void {
        checkModeChange(0, 1);
        target = deflib.constrain(Math.round(target), -32768, 32767)
        txData[22] = target & 0xFF;
        txData[23] = (target >> 8) & 0xFF;
        txData[21] = deflib.constrain(velocity, 10, 100);
    }

    function waitForRotationTarget(target: number): void {
        let start = input.runningTime()
        let targetHeading = normalizeHeading360(target)

        while ((input.runningTime() - start) < rotationTimeoutMs) {
            let currentHeading = normalizeHeading360(controlYaw)
            if (Math.abs(getShortestHeadingDelta(currentHeading, targetHeading)) <= rotationToleranceDeg)
                break
            basic.pause(rotationPollMs)
        }

        clearRotationCommand()
    }

    function executeRotationTarget(target: number, velocity: number, waitForCompletion: boolean): void {
        if (Math.round(target) == Math.round(controlYaw)) {
            clearRotationCommand()
            return
        }

        writeRotationTarget(target, velocity)
        if (waitForCompletion)
            waitForRotationTarget(target)
    }

    function planRelativeRotationTarget(dir: deflib.RotateDirection, deg: number): number {
        let angle = sanitizeRotationAngle(deg);
        let delta = (dir == deflib.RotateDirection.Clockwise) ? angle : -1 * angle;
        return controlYaw + delta;
    }

    function planAbsoluteRotationTarget(angle: number): number {
        let currentHeading = normalizeHeading360(controlYaw);
        let targetHeading = normalizeHeading360(sanitizeRotationAngle(angle));
        let delta = getShortestHeadingDelta(currentHeading, targetHeading);
        return controlYaw + delta;
    }

    //% blockId=robodog_headled_image_literal_v2
    //% block="headled image"
    //% blockHidden=true
    //% shim=images::createImage
    //% imageLiteral=1
    //% imageLiteralColumns=8
    //% imageLiteralRows=8
    export function headLedImageLiteral(data: string): string {
        return data;
    }

    function encodeHeadLedImage(data: Image): number[] {
        let encoded = [0, 0, 0, 0, 0, 0, 0, 0];
        for (let y = 0; y < 8; y++) {
            let row = 0;
            for (let x = 0; x < 8; x++) {
                if (data.pixel(x, y))
                    row |= 0x80 >> x;
            }
            encoded[y] = row;
        }
        return encoded;
    }

    //% blockId=robodog_rf_band
    //% block="set radio band to $band"
    //% band.min=0 band.max=79 band.defl=7
    //% group="Connection"
    //% weight=109
    export function rfBand(band: number): void {
        if (radioInit)
            return;
        band = deflib.constrain(band, 0, 79);
        radio.setGroup(14)
        radio.setFrequencyBand(band);
        radioInit = true;
        radioTxIndex = 0;
    }

    //% block="take $action posture with Robodog"
    //% group="Motion"
    //% weight=100
    export function gesture(action: deflib.Posture): void {
        checkModeChange(0, 4);
        txData[16] = deflib.constrain(action, 0, 4)
    }

    //% blockId=robodog_leg_bend
    //% block="set Robodog $legs walking height to $height"
    //% legs.defl=deflib.LegGroup.AllLegs
    //% height.min=0 height.max=100 height.defl=70
    //% group="Motion"
    //% weight=99
    export function legBend(legs: deflib.LegGroup, height: number): void {
        checkModeChange(0, 1);
        height = normalizeUserRangeToInt(height, 0, 100, 20, 90);
        if (legs == 0)
            txData[16] = txData[17] = txData[18] = txData[19] = height;
        if (legs == 1)
            txData[16] = txData[19] = height;
        if (legs == 2)
            txData[17] = txData[18] = height;
        if (legs == 3)
            txData[16] = txData[17] = height;
        if (legs == 4)
            txData[18] = txData[19] = height;
    }

    //% block="set Robodog $leg leg height to $height and foot forward/backward to $fb"
    //% height.min=0 height.max=100 height.defl=70
    //% fb.min=-100 fb.max=100 fb.defl=0
    //% inlineInputMode=inline
    //% group="Motion"
    //% weight=98
    export function leg(leg: deflib.LegSelection, height: number, fb: number): void {
        checkModeChange(-127, 2);
        height = normalizeUserRangeToInt(height, 0, 100, 20, 90);
        fb = normalizeUserRangeToInt(fb, -100, 100, -90, 90);

        let _pos = legPos[leg];
        for (let n = 0; n < 4; n++) {
            if (_pos[n] == 1) {
                txData[16 + n * 2] = height;
                txData[16 + n * 2 + 1] = fb;
            }
        }
    }

    //% block="set Robodog $leg shoulder to $deg1 degrees and knee to $deg2 degrees"
    //% deg1.min=-90 deg1.max=90 deg1.defl=0
    //% deg2.min=-90 deg2.max=70 deg1.def2=0
    //% inlineInputMode=inline
    //% group="Motion"
    //% weight=97
    export function motor(leg: deflib.LegSelection, deg1: number, deg2: number): void {
        checkModeChange(-127, 3);

        deg1 = deflib.constrain(deg1, -90, 90);
        deg2 = deflib.constrain(deg2, -90, 90);

        let _pos = legPos[leg];
        for (let n = 0; n < 4; n++) {
            if (_pos[n] == 1) {
                txData[16 + n * 2] = deg1;
                txData[16 + n * 2 + 1] = deg2;
            }
        }
    }

    //% block="move Robodog $dir at speed $velocity"
    //% velocity.min=0 velocity.max=100 velocity.defl=50
    //% group="Motion"
    //% weight=96
    export function move(dir: deflib.MoveDirection, velocity: number): void {
        checkModeChange(0, 1);
        velocity = deflib.constrain(velocity, -100, 100);
        txData[20] = (dir == deflib.MoveDirection.Forward) ? velocity : -1 * velocity;
    }

    //% block="turn Robodog $dir by $deg degrees at speed $velocity"
    //% deg.min=0 deg.max=360 deg.defl=90
    //% velocity.min=0 velocity.max=100 velocity.defl=100
    //% inlineInputMode=inline
    //% group="Motion"
    //% weight=95
    export function rotation(dir: deflib.RotateDirection, deg: number, velocity: number): void {
        let target = planRelativeRotationTarget(dir, deg);
        executeRotationTarget(target, velocity, rotationWaitRelative);
    }

    //% blockId=robodog_rotation_absolute
    //% block="return Robodog to start direction"
    //% angle.min=0 angle.max=360 angle.defl=0
    //% velocity.min=10 velocity.max=100 velocity.defl=100
    //% group="Motion"
    //% weight=94
    export function rotationAbsolute(angle: number = 0, velocity: number = 100): void {
        let target = planAbsoluteRotationTarget(angle);
        executeRotationTarget(target, velocity, rotationWaitAbsolute);
    }

    //% blockId=robodog_headled_exp
    //% block="show $exp expression on Robodog head LED"
    //% group="LED"
    //% weight=89
    export function headLedExp(exp: deflib.LedExpression): void {
        prepareHeadLedPayload(0x82);
        txData[24] = exp;
        stageHeadLedPayload();
    }

    //% blockId=robodog_headled_print
    //% block="show character $character on Robodog $what head LED"
    //% character.defl="A"
    //% group="LED"
    //% weight=88
    export function headLedPrint(what: deflib.HeadLedSide, character: string): void {
        prepareHeadLedPayload(0x83);
        let aa = character.charCodeAt(0);
        let offsets = getHeadLedOffsets(what);
        for (let offset of offsets)
            txData[24 + offset] = aa;
        stageHeadLedPayload();
    }

    //% blockId=robodog_headled_draw_matrix
    //% block="show on Robodog $what head LED|$data"
    //% inlineInputMode=external
    //% data.shadow=robodog_headled_image_literal_v2
    //% group="LED"
    //% weight=87
    export function headLedDraw(what: deflib.HeadLedSide, data: string): void {
        prepareHeadLedPayload(0x81);
        let image = <Image><any>data;
        let encoded = encodeHeadLedImage(image);
        let offsets = getHeadLedOffsets(what);
        for (let offset of offsets) {
            for (let n = 0; n < 8; n++)
                txData[24 + offset + n] = encoded[n];
        }
        stageHeadLedPayload();
    }

    //% blockId=robodog_bodyled
    //% block="set Robodog body LED color to R:$r, G:$g, B:$b"
    //% r.min=0 r.max=255 r.defl=255
    //% g.min=0 g.max=255 g.defl=255
    //% b.min=0 b.max=255 b.defl=255
    //% inlineInputMode=inline
    //% group="LED"
    //% weight=85
    export function bodyLed(r: number, g: number, b: number): void {
        txData[24] = deflib.constrain(r, 0, 255);
        txData[25] = deflib.constrain(g, 0, 255);
        txData[26] = deflib.constrain(b, 0, 255);

        txData[28] = txData[32] = txData[36] = txData[24];
        txData[29] = txData[33] = txData[37] = txData[25];
        txData[30] = txData[34] = txData[38] = txData[26];
        txData[14] = (txData[14] & 0xC0) | 0x44;
        ledData[1] = txData[14];
        ledData[0] = ledData[0] | 0x40;
        for (let n = 0; n < 16; n++)
            ledData[n + 18] = txData[n + 24];
    }

    //% blockId=robodog_sound_play
    //% block="play sound effect $what at $volume volume"
    //% group="Sound"
    //% weight=79
    export function soundPlay(what: deflib.SoundEffect, volume: deflib.SoundVolume): void {
        let id = (txData[7] & 0x80) == 0x80 ? 0x00 : 0x80;
        txData[7] = what | id;
        txData[8] = volume;
    }

    //% blockId=robodog_ai_detection
    //% block="run AI $what"
    //% group="AI Settings"
    //% weight=69
    export function aiDetection(what: deflib.AiMode): void {
        extTxData[7] = what | 0x10;
        isExtPacketEnabled = true;
    }

    //% blockId=robodog_face_tracking
    //% block="track $what face"
    //% group="AI Settings"
    //% weight=68
    export function faceTracking(what: deflib.AiClass): void {
        extTxData[7] = 0x12;
        extTxData[8] = what | 0x30;
        isExtPacketEnabled = true;
    }

    //% blockId=robodog_get_button
    //% block="button is pressed"
    //% group="Sensors"
    //% weight=59
    export function getButton(): boolean {
        return buttonPressed;
    }

    //% blockId=robodog_get_battery
    //% block="battery (\\%)"
    //% group="Sensors"
    //% weight=58
    export function getBattery(): number {
        return battery;
    }

    //% blockId=robodog_get_tof
    //% block="distance sensor"
    //% group="Sensors"
    //% weight=57
    export function getTof(): number {
        return tof;
    }

    //% blockId=robodog_get_tilt
    //% block="read tilt as $what"
    //% group="Sensors"
    //% weight=56
    export function getTilt(what: deflib.TiltAxis): number {
        return what == deflib.TiltAxis.LeftRight ? roll : pitch;
    }

    //% blockId=robodog_get_rotation
    //% block="robodog heading (˚)"
    //% group="Sensors"
    //% weight=55
    export function getRotation(): number {
        return yaw;
    }

    //% blockId=robodog_get_camera_alive
    //% block="AI camera ready"
    //% group="AI Data"
    //% weight=49
    export function getCameraAlive(): boolean {
        return cameraAlive;
    }

    //% blockId=robodog_get_face_class
    //% block="face detection class"
    //% group="AI Data"
    //% weight=48
    export function getFaceClass(): number {
        if (aiData[0] != 1)
            return 0;
        return aiData[1];
    }

    //% blockId=robodog_get_face_tracking
    //% block="face tracking"
    //% group="AI Data"
    //% weight=47
    export function getFaceTracking(): number {
        if (aiData[0] != 2)
            return 0;
        return aiData[1];
    }

    //% blockId=robodog_get_color_detect
    //% block="color detection class"
    //% group="AI Data"
    //% weight=46
    export function getColorDetect(): number {
        if (aiData[0] != 3)
            return 0;
        return aiData[1];
    }

    //% blockId=robodog_get_qr_code
    //% block="QR code value"
    //% group="AI Data"
    //% weight=45
    export function getQrCode(): string {
        if (aiData[0] != 4)
            return "none";
        if (aiData[1] != 1)
            return "none";

        let qrLength = 0;
        for (let i = 0; i < 8; i++) {
            if (aiData[2 + i] == 0)
                break
            qrLength += 1;
        }
        return aiData.slice(2, qrLength).toString();
    }

    //% blockId=robodog_get_ai_position
    //% block="recognized result $what position"
    //% group="AI Data"
    //% weight=44
    export function getAiPosition(what: deflib.AiPositionAxis): number {
        let val = what == deflib.AiPositionAxis.X ? aiData[2] : aiData[3];
        return val > 127 ? val - 256 : val;
    }
}
