param(
    [switch]$Update,
    [switch]$NoGolden,
    [string]$Weft
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
if (-not $Weft) {
    $Weft = Join-Path $root 'build\bin\Release\weft.exe'
}
$Weft = (Resolve-Path -LiteralPath $Weft).Path
$goldenPath = Join-Path $PSScriptRoot 'golden_counts.txt'
$outputDir = Join-Path ([IO.Path]::GetTempPath()) (
    'weft-corpus-gate-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $outputDir | Out-Null

$counts = [System.Collections.Generic.List[string]]::new()
$failures = [System.Collections.Generic.List[string]]::new()

function Invoke-GateCase {
    param(
        [string]$Name,
        [string]$InputFile,
        [string[]]$ProfileArgs,
        [string]$ProfileTag,
        [bool]$RequireWatertight
    )

    $objPath = Join-Path $outputDir "$Name-$ProfileTag.obj"
    $args = @('mesh', $InputFile, '-o', $objPath) +
            $ProfileArgs + @('--validate')
    $lines = @(& $Weft @args 2>&1)
    $exitCode = $LASTEXITCODE
    $logPath = Join-Path $outputDir "$Name-$ProfileTag.log"
    $lines | Set-Content -LiteralPath $logPath

    if ($RequireWatertight -and $exitCode -ne 0) {
        $failures.Add("$Name [$ProfileTag]: exit $exitCode (see $logPath)")
    }
    if ($RequireWatertight -and
        -not ($lines -match 'watertight:\s+yes')) {
        $failures.Add("$Name [$ProfileTag]: not watertight")
    }

    $stats = $null
    foreach ($line in $lines) {
        if ($line -match '(\d+) quads, (\d+) tris, (\d+) n-gons') {
            $stats = "$($matches[1]) quads, $($matches[2]) tris, " +
                     "$($matches[3]) n-gons"
            break
        }
    }
    if (-not $stats) {
        $failures.Add("$Name [$ProfileTag]: polygon counts missing")
        $stats = 'counts missing'
    }
    $counts.Add("$Name $ProfileTag $stats")

    $demotion = $lines | Where-Object { $_ -match 'demoted:' } |
                Select-Object -First 1
    if ($demotion -and
        $demotion -notmatch
            '0 to raw triangulation, 0 emitted nothing') {
        $failures.Add("$Name [$ProfileTag]: $demotion")
    }
}

try {
    $fixtures = @(
        'cylinder', 'box', 'cone', 'sphere', 'torus', 'fillet', 'hole',
        'demo', 'boss', 'notched', 'slotted', 'barrel', 'drilled',
        'bossfillet', 'ribbon', 'ribbonnotch', 'hairline', 'canrev',
        'microedge', 'filletslot', 'torture', 'slitdrill'
    )
    foreach ($fixture in $fixtures) {
        $stepPath = Join-Path $outputDir "$fixture.step"
        & $Weft fixture $stepPath --shape $fixture *> $null
        if ($LASTEXITCODE -ne 0) {
            $failures.Add("${fixture}: fixture generation failed")
            continue
        }
        $requireWatertight = $fixture -ne 'slitdrill'
        Invoke-GateCase $fixture $stepPath @('--profile', 'cad') 'cad' `
            $requireWatertight
        Invoke-GateCase $fixture $stepPath @() 'default' $requireWatertight
    }

    $trackedCorpus = @(
        git -C $root ls-files 'tests/STEP_Examples/*.stp'
    )
    foreach ($relativePath in $trackedCorpus) {
        $name = [IO.Path]::GetFileNameWithoutExtension($relativePath)
        $inputPath = Join-Path $root $relativePath
        $requireWatertight = $name -ne 'tork'
        Invoke-GateCase $name $inputPath @('--profile', 'cad') 'cad' `
            $requireWatertight
        Invoke-GateCase $name $inputPath @() 'default' $requireWatertight
    }

    if ($Update) {
        $counts | Set-Content -LiteralPath $goldenPath
        Write-Output "golden counts updated: $goldenPath"
    } elseif ($NoGolden) {
        Write-Output 'golden count diff skipped (invariants-only run)'
    } elseif (Test-Path -LiteralPath $goldenPath) {
        $golden = @(Get-Content -LiteralPath $goldenPath)
        if ($golden.Count -ne $counts.Count) {
            $failures.Add(
                "golden rows $($golden.Count) != actual $($counts.Count)")
        } else {
            for ($i = 0; $i -lt $golden.Count; ++$i) {
                if ($golden[$i] -ne $counts[$i]) {
                    $failures.Add(
                        "golden[$i] expected '$($golden[$i])' " +
                        "actual '$($counts[$i])'")
                }
            }
        }
    } else {
        Write-Output 'note: no golden table; rerun with -Update to create it'
    }

    foreach ($failure in $failures) {
        Write-Output "FAIL $failure"
    }
    if ($failures.Count -eq 0) {
        Write-Output "corpus gate: PASS ($($counts.Count) cases)"
    } else {
        Write-Output "corpus gate: FAIL ($($failures.Count) failures)"
        exit 1
    }
} finally {
    Remove-Item -LiteralPath $outputDir -Recurse -Force -ErrorAction SilentlyContinue
}
