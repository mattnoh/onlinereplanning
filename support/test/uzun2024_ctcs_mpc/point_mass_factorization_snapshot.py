"""Copy a solver into a NEW directory; add logging without changing its map."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def snapshot(source, output):
    output.mkdir(parents=True, exist_ok=False)
    for name in ('alipddp', 'problem_maker'):
        shutil.copytree(source/name, output/name)
    (output/'problem_examples').mkdir()
    shutil.copy2(source/'problem_examples/point_mass_ctcs_compare.cpp',
                 output/'problem_examples/point_mass_ctcs_compare.cpp')
    target = output/'alipddp/alipddp/alipddp.tpp'
    before = target.read_text()
    anchor = '                const bool finite = hat_Quu.allFinite();'
    if before.count(anchor) != 1:
        raise ValueError('source changed: expected exactly one diagnostic anchor')
    addition = r'''
                // Logging only: emit the actual factored matrix and contributions.
                std::ostringstream matrix_log;
                matrix_log << std::setprecision(17) << "[QUU_MATRIX] {\"stage\":" << k;
                auto emit_matrix = [&](const char* name, const Eigen::MatrixXd& a) {
                    matrix_log << ",\"" << name << "\":[";
                    for (int i=0;i<a.rows();++i) {
                        if(i) matrix_log << ',';
                        matrix_log << '[';
                        for(int j=0;j<a.cols();++j) {
                            if(j) matrix_log << ',';
                            matrix_log << a(i,j);
                        }
                        matrix_log << ']';
                    }
                    matrix_log << ']';
                };
                emit_matrix("base", hat_Quu);
                emit_matrix("factored", hat_Quu_factored);
                emit_matrix("uncondensed", Quu);
                if(is_c_active[k]) emit_matrix("ipm", hat_Quu_ipm_all[k]);
                if(is_ec_active[k]) emit_matrix("alm", hat_Quu_alm_all[k]);
                matrix_log << '}';
                std::cout << matrix_log.str() << std::endl;
'''
    target.write_text(before.replace(anchor, anchor+addition))
    manifest = dict(source=str(source.resolve()),
                    original_tpp_sha256=hashlib.sha256(before.encode()).hexdigest(),
                    modified_tpp_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                    change='logging only inside existing SW_QUU_DIAG failure branch',
                    files={str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in sorted(output.rglob('*')) if p.is_file()})
    (output/'snapshot_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def legacy_soc_snapshot(source, output):
    """Research-only old SOC equations; leave production NT unchanged.

    Keep the current gate flag and every other solver equation identical, so
    old/NT x legacy/strict can be compared in one pinned source snapshot.
    """
    output.mkdir(parents=True, exist_ok=False)
    for name in ('alipddp', 'problem_maker'):
        shutil.copytree(source/name, output/name)
    (output/'problem_examples').mkdir()
    shutil.copy2(source/'problem_examples/point_mass_ctcs_compare.cpp',
                 output/'problem_examples/point_mass_ctcs_compare.cpp')
    target=output/'alipddp/helper_functions/soc_helper.h'
    before=target.read_text()
    start=before.index('inline void ntElimination(')
    end=before.index('// === END LLM-AUTHORED ===',start)
    declaration="""inline void ntElimination(Eigen::Ref<Eigen::MatrixXd>,
    Eigen::Ref<Eigen::VectorXd>,const Eigen::VectorXd&,const Eigen::VectorXd&,
    const Eigen::VectorXd&,double);
"""
    definition="""inline void ntElimination(Eigen::Ref<Eigen::MatrixXd> metric,
                          Eigen::Ref<Eigen::VectorXd> rhs,
                          const Eigen::VectorXd& s, const Eigen::VectorXd& y,
                          const Eigen::VectorXd& rp, double mu) {
    // RESEARCH ONLY: invoke the exact original Jordan helper operations.
    Eigen::VectorXd rd(s.size()),r(s.size());
    LtimesVec(rd,y,s);rd(0)-=mu;
    LtimesVec(r,y,rp);r-=rd;
    LinvTimesVec(rhs,s,r);
    LinvTimesArrow(metric,s,y);
}
"""
    modified=before[:start]+declaration+before[end:]
    anchor='} // namespace soc_helper'
    if modified.count(anchor)!=1:raise ValueError('unexpected namespace layout')
    target.write_text(modified.replace(anchor,definition+anchor))
    manifest=dict(source=str(source.resolve()), change='research-only old SOC elimination; current gate flag retained',
                  original_helper_sha256=hashlib.sha256(before.encode()).hexdigest(),
                  files={str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest()
                         for p in sorted(output.rglob('*')) if p.is_file()})
    (output/'snapshot_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--legacy-soc', action='store_true', help='isolated old-SOC ablation, no factorization logging patch')
    args = parser.parse_args()
    prepare = legacy_soc_snapshot if args.legacy_soc else snapshot
    print(json.dumps({k: v for k, v in prepare(args.source, args.output).items()
                      if k != 'files'}, indent=2))
