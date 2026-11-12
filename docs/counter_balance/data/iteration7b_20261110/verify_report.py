'''Verify local report structure, numeric tables and evidence scope'''
import json
from pathlib import Path
from html.parser import HTMLParser
from urllib.parse import unquote, urlsplit


class Page(HTMLParser):
    def __init__(self):
        super().__init__()
        self.stack, self.links, self.labels = [], [], []
        self.ids = set()

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if 'id' in attrs:
            assert attrs['id'] not in self.ids
            self.ids.add(attrs['id'])
        self.labels.extend(attrs.get('aria-labelledby', '').split())
        if 'href' in attrs:
            self.links.append(attrs['href'])
        if tag not in ('meta', 'link', 'br', 'hr', 'img', 'input', 'wbr'):
            self.stack.append(tag)

    def handle_endtag(self, tag):
        assert self.stack and self.stack.pop() == tag, tag


def parse(path):
    page = Page()
    page.feed(path.read_text())
    assert not page.stack
    assert set(page.labels) <= page.ids
    return page


def main():
    here = Path(__file__).resolve().parent
    directory = here.parents[1]
    paths = [directory / 'counter_balance_analysis_iteration_7b.html',
             directory / 'counter_balance_iteration_7b.html',
             directory / '../../../../docs/iteration7b_hardware_validation.html']
    for path in paths:
        page = parse(path)
        for link in page.links:
            url = urlsplit(link)
            if url.scheme:
                continue
            target = path.parent / unquote(url.path) if url.path else path
            assert target.exists(), (path, link)
            if url.fragment and target.suffix == '.html':
                assert unquote(url.fragment) in parse(target).ids, link
        print(f'PASS HTML: {path.name}')
    text = paths[0].read_text()
    assert not any(word in text for word in ('Analysis in progress', 'results pending', 'coverage pending'))
    data = json.loads((here / 'results.json').read_text())
    assert len(data['records']) == 12 and len(data['pairs']) == 6
    assert sum(row['official']['sample_count'] for row in data['records']) == 4057
    for row in data['records']:
        for key, scale in (('move_rms_tilt_deviation_deg', 1),
                           ('timed_position_error_rms_m', 1000),
                           ('frozen_uniform_rms_m', 1000), ('residual_drift_deg', 1)):
            assert f'{row["official"][key]*scale:.3f}' in text, (row['run'], key)
    experiment = json.loads((here / 'recovery_prototype/summary.json').read_text())
    assert experiment['full_simulation_cells_used'] == 1
    assert experiment['full_simulation_candidate_cells'] == 0
    assert experiment['synthetic_actuator_cells'] == 8
    assert 'not a robust control implementation' in text
    print('PASS: printed hardware values, 12-run/6-pair counts, no full-body candidate claim')


if __name__ == '__main__':
    main()
