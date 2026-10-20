// Move and resize the X-Plane window by dragging it with synthetic mouse
// events (X-Plane's window isn't reachable through Accessibility/AppleScript).
//
//   swift scripts/demo/xplane_window.swift probe              # print its frame
//   swift scripts/demo/xplane_window.swift 0 30 2456 1410     # x y width height, points
//
// 2456x1410 at (0, 30) fills the left half of a 5120x1440 display with a
// 16:9 picture below the 28 pt title bar. Re-run obs_setup.py afterwards.
import CoreGraphics
import Foundation

func xplaneBounds() -> CGRect? {
    let info = CGWindowListCopyWindowInfo([.optionOnScreenOnly, .excludeDesktopElements], kCGNullWindowID) as! [[String: Any]]
    var best: CGRect?
    for w in info where (w[kCGWindowOwnerName as String] as? String)?.hasPrefix("X-Plane") == true
        && (w[kCGWindowLayer as String] as? Int) == 0 {
        if let b = w[kCGWindowBounds as String] as? [String: CGFloat] {
            let r = CGRect(x: b["X"]!, y: b["Y"]!, width: b["Width"]!, height: b["Height"]!)
            if r.width * r.height > (best.map { $0.width * $0.height } ?? 0) { best = r }
        }
    }
    return best
}

func drag(from a: CGPoint, to b: CGPoint) {
    let src = CGEventSource(stateID: .hidSystemState)
    CGEvent(mouseEventSource: src, mouseType: .mouseMoved, mouseCursorPosition: a, mouseButton: .left)!.post(tap: .cghidEventTap)
    usleep(150_000)
    CGEvent(mouseEventSource: src, mouseType: .leftMouseDown, mouseCursorPosition: a, mouseButton: .left)!.post(tap: .cghidEventTap)
    usleep(150_000)
    for i in 1...30 {
        let t = CGFloat(i) / 30
        let p = CGPoint(x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t)
        CGEvent(mouseEventSource: src, mouseType: .leftMouseDragged, mouseCursorPosition: p, mouseButton: .left)!.post(tap: .cghidEventTap)
        usleep(15_000)
    }
    usleep(150_000)
    CGEvent(mouseEventSource: src, mouseType: .leftMouseUp, mouseCursorPosition: b, mouseButton: .left)!.post(tap: .cghidEventTap)
    usleep(600_000)
}

let args = CommandLine.arguments
guard var r = xplaneBounds() else { print("no X-Plane window"); exit(1) }
print("before: \(r)")
if args.count > 1 && args[1] == "probe" { exit(0) }
let (tx, ty, tw, th) = (CGFloat(Double(args[1])!), CGFloat(Double(args[2])!), CGFloat(Double(args[3])!), CGFloat(Double(args[4])!))
// Move by the title bar (left of center, clear of the title text's buttons).
drag(from: CGPoint(x: r.midX - 200, y: r.minY + 12), to: CGPoint(x: tx + r.width / 2 - 200, y: ty + 12))
r = xplaneBounds()!
print("moved: \(r)")
// Resize by the bottom-right corner.
drag(from: CGPoint(x: r.maxX - 2, y: r.maxY - 2), to: CGPoint(x: tx + tw - 2, y: ty + th - 2))
print("after: \(xplaneBounds()!)")
