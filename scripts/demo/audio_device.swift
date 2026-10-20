// List or set the macOS default audio devices.
//
//   swift scripts/demo/audio_device.swift                 # list devices and defaults
//   swift scripts/demo/audio_device.swift input HyperX    # default mic (push-to-talk uses it)
//   swift scripts/demo/audio_device.swift output HyperX   # default output (radio voices play there)
//
// The name is a case-insensitive substring; the first device with channels in
// that direction wins.
import CoreAudio
import Foundation

func address(_ selector: AudioObjectPropertySelector,
             _ scope: AudioObjectPropertyScope = kAudioObjectPropertyScopeGlobal) -> AudioObjectPropertyAddress {
    AudioObjectPropertyAddress(mSelector: selector, mScope: scope, mElement: kAudioObjectPropertyElementMain)
}

func devices() -> [AudioDeviceID] {
    var addr = address(kAudioHardwarePropertyDevices)
    var size: UInt32 = 0
    AudioObjectGetPropertyDataSize(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil, &size)
    var ids = [AudioDeviceID](repeating: 0, count: Int(size) / MemoryLayout<AudioDeviceID>.size)
    AudioObjectGetPropertyData(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil, &size, &ids)
    return ids
}

func name(_ id: AudioDeviceID) -> String {
    var addr = address(kAudioObjectPropertyName)
    var cf: Unmanaged<CFString>?
    var size = UInt32(MemoryLayout<Unmanaged<CFString>?>.size)
    AudioObjectGetPropertyData(id, &addr, 0, nil, &size, &cf)
    return cf?.takeRetainedValue() as String? ?? "?"
}

func channels(_ id: AudioDeviceID, input: Bool) -> Int {
    var addr = address(kAudioDevicePropertyStreamConfiguration,
                       input ? kAudioDevicePropertyScopeInput : kAudioDevicePropertyScopeOutput)
    var size: UInt32 = 0
    guard AudioObjectGetPropertyDataSize(id, &addr, 0, nil, &size) == noErr, size > 0 else { return 0 }
    let list = UnsafeMutableRawPointer.allocate(byteCount: Int(size), alignment: 16)
    defer { list.deallocate() }
    AudioObjectGetPropertyData(id, &addr, 0, nil, &size, list)
    let buffers = UnsafeMutableAudioBufferListPointer(list.assumingMemoryBound(to: AudioBufferList.self))
    return buffers.reduce(0) { $0 + Int($1.mNumberChannels) }
}

func defaultDevice(_ selector: AudioObjectPropertySelector) -> AudioDeviceID {
    var addr = address(selector)
    var id = AudioDeviceID(0)
    var size = UInt32(MemoryLayout<AudioDeviceID>.size)
    AudioObjectGetPropertyData(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil, &size, &id)
    return id
}

func setDefault(_ selector: AudioObjectPropertySelector, _ id: AudioDeviceID) -> Bool {
    var addr = address(selector)
    var id = id
    return AudioObjectSetPropertyData(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil,
                                      UInt32(MemoryLayout<AudioDeviceID>.size), &id) == noErr
}

let args = CommandLine.arguments.dropFirst()
if args.isEmpty {
    let (inp, out) = (defaultDevice(kAudioHardwarePropertyDefaultInputDevice),
                      defaultDevice(kAudioHardwarePropertyDefaultOutputDevice))
    for id in devices() {
        let (i, o) = (channels(id, input: true), channels(id, input: false))
        let marks = (id == inp ? " [default input]" : "") + (id == out ? " [default output]" : "")
        print("\(name(id))  in:\(i) out:\(o)\(marks)")
    }
    exit(0)
}
guard args.count == 2, let dir = args.first, dir == "input" || dir == "output" else {
    FileHandle.standardError.write("usage: audio_device.swift [input|output <name>]\n".data(using: .utf8)!)
    exit(2)
}
let isInput = dir == "input"
let needle = args.last!.lowercased()
guard let id = devices().first(where: {
    name($0).lowercased().contains(needle) && channels($0, input: isInput) > 0
}) else {
    FileHandle.standardError.write("no \(dir) device matching \(needle)\n".data(using: .utf8)!)
    exit(1)
}
let selectors = isInput ? [kAudioHardwarePropertyDefaultInputDevice]
                        : [kAudioHardwarePropertyDefaultOutputDevice, kAudioHardwarePropertyDefaultSystemOutputDevice]
for s in selectors where !setDefault(s, id) {
    FileHandle.standardError.write("couldn't set the default \(dir)\n".data(using: .utf8)!)
    exit(1)
}
print("default \(dir): \(name(id))")
