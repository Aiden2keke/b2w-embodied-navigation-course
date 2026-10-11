#!/usr/bin/python3
"""Convert the pinned B2-W TorchScript policy; check ONNX parity before replacing it."""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import numpy as np
import onnx
import onnxruntime as ort
import torch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    args = parser.parse_args()
    directory = args.bundle / 'policy/b2w/robot_lab'
    source = directory / 'policy.pt'
    expected = '38155076408e8eccb22690c6c5be14bd1dcb9149245ca5e493308a9f6ff93b34'
    if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
        raise ValueError('The TorchScript weight differs from the pinned B2-W policy')
    torch.set_num_threads(1)
    policy = torch.jit.load(str(source), map_location='cpu').eval()
    with tempfile.TemporaryDirectory(dir=directory) as temporary:
        output = Path(temporary) / 'policy.onnx'
        torch.onnx.export(policy, torch.zeros(1, 57), str(output), opset_version=17,
                          input_names=['observations'], output_names=['actions'],
                          dynamic_axes={'observations': {0: 'batch'}, 'actions': {0: 'batch'}})
        onnx.checker.check_model(onnx.load(str(output)))
        session = ort.InferenceSession(str(output), providers=['CPUExecutionProvider'])
        observations = np.random.default_rng(0).normal(size=(32, 57)).astype(np.float32)
        with torch.inference_mode():
            reference = policy(torch.from_numpy(observations)).numpy()
        actual = session.run(None, {'observations': observations})[0]
        np.testing.assert_allclose(actual, reference, rtol=1e-5, atol=1e-5)
        error = float(np.max(np.abs(actual-reference)))
        output.replace(directory / 'policy.onnx')
    provenance = args.bundle / 'POLICY_SOURCE.json'
    metadata = json.loads(provenance.read_text())
    data = (directory / 'policy.onnx').read_bytes()
    metadata['files']['policy/b2w/robot_lab/policy.onnx'] = {
        'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
        'derived_from': 'policy.pt', 'conversion': f'PyTorch {torch.__version__}, ONNX opset 17',
        'parity_max_absolute_error': error}
    provenance.write_text(json.dumps(metadata, indent=2)+'\n')
    print(f'Exported {directory / "policy.onnx"}; maximum parity error {error:.3g}')


if __name__ == '__main__':
    main()
