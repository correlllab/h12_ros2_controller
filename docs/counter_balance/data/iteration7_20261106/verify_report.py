'''Validate retained report links, HTML structure and numerical evidence coverage'''
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
            assert attrs['id'] not in self.ids, attrs['id']
            self.ids.add(attrs['id'])
        self.labels.extend(attrs.get('aria-labelledby', '').split())
        if 'href' in attrs:
            self.links.append(attrs['href'])
        if tag not in ('meta', 'link', 'br', 'img', 'input', 'hr', 'wbr'):
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
    for name in ('counter_balance_analysis_iteration_7_hardware.html',
                 'counter_balance_iteration_7b.html', 'counter_balance_iteration_7.html'):
        path = directory / name
        page = parse(path)
        for link in page.links:
            url = urlsplit(link)
            if url.scheme in ('http', 'https', 'mailto'):
                continue
            target = directory / unquote(url.path) if url.path else path
            assert target.exists(), (name, link)
            if url.fragment and target.suffix == '.html':
                assert unquote(url.fragment) in parse(target).ids
        assert 'Analysis in progress' not in path.read_text()
        assert 'Pending verified' not in path.read_text()
        print(f'PASS: {name}, tags, IDs, accessibility labels and links')
    records = json.loads((here / 'results.json').read_text())['records']
    assert len(records) == 12
    report = (directory / 'counter_balance_analysis_iteration_7_hardware.html').read_text()
    for row in records:
        for key, scale, precision in (('frozen_uniform_rms_m', 1000, 3),
                                      ('timed_position_error_rms_m', 1000, 3),
                                      ('move_rms_tilt_deviation_deg', 1, 3),
                                      ('residual_drift_deg', 1, 3)):
            assert f'{row[key]*scale:.{precision}f}' in report, (row['run'], key)
    counter = json.loads((here / 'counter_summary.json').read_text())
    assert len(counter['runs']) == 6
    assert sum(run['phases']['all']['n'] for run in counter['runs'].values()) == 2028
    timing = json.loads((here / 'timing_summary.json').read_text())
    assert len(timing['runs']) == 12
    print('PASS: report numeric table values and retained 12-run/6-counter evidence counts')


if __name__ == '__main__':
    main()
