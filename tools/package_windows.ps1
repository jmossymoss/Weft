# Build a portable Windows zip for weft CLI + app.
# Expects a Release build under build/bin/Release (build.bat / VS generator).
param(
    [string]$BuildDir = "",
    [string]$OutDir = "",
    [string]$Name = "weft-windows-x64"
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
if (-not $BuildDir) { $BuildDir = Join-Path $Root "build" }
if (-not $OutDir) { $OutDir = Join-Path $Root "dist" }

$Bin = Join-Path $BuildDir "bin\Release"
if (-not (Test-Path (Join-Path $Bin "weft.exe"))) {
    # Ninja/single-config layout fallback
    $Bin = Join-Path $BuildDir "bin"
}
$Weft = Join-Path $Bin "weft.exe"
$App = Join-Path $Bin "weft_app.exe"
if (-not (Test-Path $Weft)) { throw "missing $Weft — build Release first" }
if (-not (Test-Path $App)) { throw "missing $App — build Release first" }

$Stage = Join-Path $OutDir $Name
if (Test-Path $Stage) { Remove-Item -Recurse -Force $Stage }
New-Item -ItemType Directory -Force -Path (Join-Path $Stage "bin") | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $Stage "blender") | Out-Null

Copy-Item $Weft (Join-Path $Stage "bin\weft.exe")
Copy-Item $App (Join-Path $Stage "bin\weft_app.exe")
Copy-Item (Join-Path $Root "blender\weft_link.py") (Join-Path $Stage "blender\weft_link.py")
Copy-Item (Join-Path $Root "README.md") (Join-Path $Stage "README.md")

# OCCT + third-party DLLs are deployed next to the exes by CMake/build.bat.
Get-ChildItem $Bin -Filter "*.dll" | ForEach-Object {
    Copy-Item $_.FullName (Join-Path $Stage "bin" $_.Name)
}

@"
Weft Windows pre-release package
================================

Contents:
  bin\weft.exe       CLI
  bin\weft_app.exe   interactive app (+ OCCT DLLs beside it)
  blender\weft_link.py

Quick start:
  bin\weft_app.exe
  bin\weft.exe fixture demo.step --shape demo
  bin\weft.exe mesh demo.step -o demo.obj --radial 12 --axial 3 --validate

Install blender\weft_link.py as a Blender add-on for the live link.

This is an engineering pre-release for testing, not an MVP package.
"@ | Set-Content -Path (Join-Path $Stage "PRERELEASE.txt") -Encoding UTF8

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$Zip = Join-Path $OutDir "$Name.zip"
if (Test-Path $Zip) { Remove-Item -Force $Zip }
Compress-Archive -Path $Stage -DestinationPath $Zip
Write-Host "wrote $Zip"
Get-Item $Zip | Format-List FullName, Length
