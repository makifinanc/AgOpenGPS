@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

:: 1) Kaynak kod klasorunu bul (once .bat'in bulundugu yer, yoksa C:\AgOpenGPS-6.8.6)
set "ROOT=%~dp0"
if not exist "%ROOT%SourceCode\AgOpenGPS.sln" (
    if exist "C:\AgOpenGPS-6.8.6\SourceCode\AgOpenGPS.sln" (
        set "ROOT=C:\AgOpenGPS-6.8.6\"
    ) else (
        echo [HATA] SourceCode\AgOpenGPS.sln bulunamadi!
        echo Bu .bat dosyasini AgOpenGPS-6.8.6 klasorunun icine koyup cift tikla.
        pause
        exit /b 1
    )
)
cd /d "%ROOT%"

:: 2) SDK 11.0.100 global.json ayari
> "global.json" echo {"sdk":{"version":"11.0.100","rollForward":"latestMajor"}}
if exist "SourceCode\global.json" (
    > "SourceCode\global.json" echo {"sdk":{"version":"11.0.100","rollForward":"latestMajor"}}
)

:: 3) Cift tanimli FieldParser.cs varsa devre disi birak
powershell -NoProfile -ExecutionPolicy Bypass -Command "$hits = @(Get-ChildItem -Path 'SourceCode\GPS' -Filter '*.cs' -Recurse | Select-String -Pattern 'class\s+AgShareFieldParser'); if ($hits.Count -gt 1 -and (Test-Path 'SourceCode\GPS\Classes\FieldParser.cs')) { Rename-Item 'SourceCode\GPS\Classes\FieldParser.cs' 'FieldParser.cs.bak' -Force; Write-Host '[OK] Cift FieldParser.cs devre disi birakildi.' -ForegroundColor Yellow }"

:: 4) Release derleme
echo.
echo [1/2] AgOpenGPS v6.8.6 (makifinanc) derleniyor...
dotnet build "SourceCode\AgOpenGPS.sln" -c Release -p:EnforceCodeStyleInBuild=false -p:RunAnalyzers=false -p:TreatWarningsAsErrors=false -p:Version=6.8.6 -p:InformationalVersion=6.8.6-makifinanc
if errorlevel 1 (
    echo.
    echo [HATA] Derleme basarisiz oldu! Yukaridaki hatayi kontrol et.
    pause
    exit /b 1
)

:: 5) FZ-G1 icin tek klasorde direkt calisir paket olustur (C:\AgOpenGPS_FZG1)
set "OUT=C:\AgOpenGPS_FZG1"
echo.
echo [2/2] FZ-G1 tasinabilir paketi hazirlaniyor: %OUT%
if exist "%OUT%" rmdir /s /q "%OUT%"
mkdir "%OUT%"

:: AgOpenGPS.exe'nin ciktigi Release klasorunu otomatik bul
set "GPS_DIR="
if exist "SourceCode\GPS\bin\Release\net48\win-x64\AgOpenGPS.exe" set "GPS_DIR=SourceCode\GPS\bin\Release\net48\win-x64\"
if not defined GPS_DIR if exist "SourceCode\GPS\bin\Release\win-x64\AgOpenGPS.exe" set "GPS_DIR=SourceCode\GPS\bin\Release\win-x64\"
if not defined GPS_DIR (
    for /f "delims=" %%F in ('dir /b /s "SourceCode\GPS\bin\Release\AgOpenGPS.exe" 2^>nul') do set "GPS_DIR=%%~dpF"
)
if not defined GPS_DIR (
    echo [HATA] Derlenen AgOpenGPS.exe bulunamadi!
    pause
    exit /b 1
)

:: Once yardimci araclari (AgIO, AgDiag, GPS_Out, ModSim, Updater) kopyala
for %%P in (
    "SourceCode\AgIO\Source\bin\Release"
    "SourceCode\AgDiag\bin\Release"
    "SourceCode\GPS_Out\Source\bin\Release"
    "SourceCode\ModSim\Source\bin\Release"
    "SourceCode\Updater\bin\Release\win-x64"
) do (
    if exist "%%~P" (
        xcopy "%%~P\*" "%OUT%\" /E /I /Y /Q >nul
    )
)

:: Ana AgOpenGPS cikti klasorunu uzerine kopyala (ana DLL'ler ve tr\ cevirileri oncelikli olsun)
xcopy "%GPS_DIR%*" "%OUT%\" /E /I /Y /Q >nul

:: Gereksiz debug dosyalarini temizle
del /q "%OUT%\*.pdb" 2>nul
del /q "%OUT%\turnline_debug.csv" 2>nul
del /q "%OUT%\youturn_debug.txt" 2>nul

echo.
echo ============================================================
echo [BASARILI] FZ-G1 icin hazir klasor olusturuldu: %OUT%
echo Bu klasoru USB ile FZ-G1 tablette C:\AgOpenGPS olarak kopyala
echo ve dogrudan AgOpenGPS.exe dosyasini calistir.
echo ============================================================
explorer "%OUT%"
pause