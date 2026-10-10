// UNTESTED SKELETON - written without Xcode access. Needs iOS 17+, a LiDAR iPhone (15 Pro / Pro Max)
// for metric scale, and "Privacy - Camera Usage Description" in Info.plist.
// Flow: guided capture around the object -> on-device photogrammetry -> OBJ in metres -> scan2cad.
import SwiftUI
import RealityKit

@MainActor
final class Capture: ObservableObject {
    @Published var session: ObjectCaptureSession?
    @Published var status = "Ready"
    private let images = FileManager.default.temporaryDirectory.appendingPathComponent("images")
    private let output = FileManager.default.temporaryDirectory.appendingPathComponent("scan.obj")

    func start() {
        guard ObjectCaptureSession.isSupported else { status = "Device not supported"; return }
        try? FileManager.default.removeItem(at: images)
        try? FileManager.default.createDirectory(at: images, withIntermediateDirectories: true)
        let s = ObjectCaptureSession()
        var cfg = ObjectCaptureSession.Configuration()
        cfg.isOverCaptureEnabled = true       // extra shots help non-LiDAR devices
        s.start(imagesDirectory: images, configuration: cfg)
        session = s
    }

    func finishAndReconstruct() async {
        session?.finish()
        status = "Reconstructing..."
        do {
            let ps = try PhotogrammetrySession(input: images)
            try ps.process(requests: [.modelFile(url: output, detail: .full)])
            for try await o in ps.outputs {
                if case .requestComplete = o { status = "Saved \(output.lastPathComponent) (metres)" }
            }
        } catch { status = "Failed: \(error.localizedDescription)" }
    }
}

struct CaptureView: View {
    @StateObject private var cap = Capture()
    var body: some View {
        ZStack {
            if let s = cap.session { ObjectCaptureView(session: s) }
            VStack {
                Spacer()
                Text(cap.status).padding(8).background(.thinMaterial)
                HStack {
                    Button("Start") { cap.start() }
                    Button("Finish") { Task { await cap.finishAndReconstruct() } }
                }.buttonStyle(.borderedProminent).padding()
            }
        }
    }
}
