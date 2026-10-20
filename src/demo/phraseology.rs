//! Radio text as it's spoken: "Skyhawk 2SP" is "Skyhawk Two Sierra
//! Papa", "runway 16R" is "runway one six right", "119.3" is "one one
//! niner point three", "the 118" is "the one-eighteen". For the radio
//! voice; the text on screen stays as written.

use crate::transcribe::keyterms::phonetic_word;

const DIGITS: [&str; 10] = ["zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "niner"];

fn digit(c: char) -> &'static str {
    DIGITS[c.to_digit(10).unwrap_or(0) as usize]
}

fn digits(s: &str) -> String {
    s.chars().filter(|c| c.is_ascii_digit()).map(digit).collect::<Vec<_>>().join(" ")
}

const ONES: [&str; 20] = [
    "zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten", "eleven", "twelve",
    "thirteen", "fourteen", "fifteen", "sixteen", "seventeen", "eighteen", "nineteen",
];
const TENS: [&str; 10] = ["", "", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety"];

/// 0..=99 in words ("eighteen", "forty-five").
fn under_hundred(n: u32) -> String {
    if n < 20 {
        ONES[n as usize].to_string()
    } else if n % 10 == 0 {
        TENS[(n / 10) as usize].to_string()
    } else {
        format!("{}-{}", TENS[(n / 10) as usize], ONES[(n % 10) as usize])
    }
}

/// An altitude the way controllers say it: "one thousand eight
/// hundred", "two thousand five hundred", "five hundred".
fn altitude(n: u32) -> String {
    let (th, hu) = (n / 1000, (n % 1000) / 100);
    let mut parts = Vec::new();
    if th > 0 {
        parts.push(format!("{} thousand", under_hundred(th)));
    }
    if hu > 0 {
        parts.push(format!("{} hundred", ONES[hu as usize]));
    }
    parts.join(" ")
}

/// The spoken form of one word (punctuation kept around it).
fn word(w: &str, prev: &str) -> String {
    let core_start = w.find(|c: char| c.is_alphanumeric()).unwrap_or(w.len());
    let core_end = w.rfind(|c: char| c.is_alphanumeric()).map(|i| i + 1).unwrap_or(core_start);
    if core_start >= core_end {
        return w.to_string();
    }
    let (pre, core, post) = (&w[..core_start], &w[core_start..core_end], &w[core_end..]);
    let prev = prev.trim_matches(|c: char| !c.is_alphanumeric()).to_ascii_lowercase();
    let spoken = spoken_token(core, &prev);
    format!("{pre}{spoken}{post}")
}

fn spoken_token(t: &str, prev: &str) -> String {
    let all_digits = t.chars().all(|c| c.is_ascii_digit());
    // Runways: "16R", "runway 34".
    if let Some(side) = t.strip_suffix(['L', 'R', 'C']) {
        if !side.is_empty() && side.len() <= 2 && side.chars().all(|c| c.is_ascii_digit()) {
            let s = match t.chars().last() {
                Some('L') => "left",
                Some('R') => "right",
                _ => "center",
            };
            return format!("{} {s}", digits(side));
        }
    }
    if all_digits && prev == "runway" {
        return digits(t);
    }
    // Frequencies: "119.3", "121.75".
    if let Some((a, b)) = t.split_once('.') {
        if a.len() == 3 && a.chars().all(|c| c.is_ascii_digit()) && !b.is_empty() && b.chars().all(|c| c.is_ascii_digit()) {
            return format!("{} point {}", digits(a), digits(b));
        }
    }
    // Callsign suffixes: "2SP", "123AB", "N172SP".
    let body = t.strip_prefix('N').filter(|r| r.starts_with(|c: char| c.is_ascii_digit())).unwrap_or(t);
    let has_digit = body.chars().any(|c| c.is_ascii_digit());
    let has_alpha = body.chars().any(|c| c.is_ascii_alphabetic());
    if has_digit && has_alpha && body.chars().all(|c| c.is_ascii_alphanumeric()) && body.starts_with(|c: char| c.is_ascii_digit()) {
        let mut out: Vec<String> = Vec::new();
        if body.len() != t.len() {
            out.push("November".into());
        }
        for c in body.chars() {
            if c.is_ascii_digit() {
                out.push(capitalize(digit(c)));
            } else if let Some(p) = phonetic_word(c) {
                out.push(capitalize(p));
            }
        }
        return out.join(" ");
    }
    if all_digits {
        let n: u32 = t.parse().unwrap_or(0);
        // A 360 / 180 / 270.
        if (prev == "a" || prev == "right" || prev == "left") && matches!(n, 180 | 270 | 360) {
            return match n {
                180 => "one-eighty",
                270 => "two-seventy",
                _ => "three-sixty",
            }
            .to_string();
        }
        // Freeways: "the 118", "the 101", "the 405".
        if prev == "the" && (2..=3).contains(&t.len()) {
            return match t.len() {
                2 => under_hundred(n),
                _ if n % 100 < 10 => format!("{}-oh-{}", ONES[(n / 100) as usize], ONES[(n % 10) as usize]),
                _ => format!("{}-{}", ONES[(n / 100) as usize], under_hundred(n % 100)),
            };
        }
        // Headings: "heading 230", "turn left 090".
        if matches!(prev, "heading" | "to") && t.len() == 3 {
            return digits(t);
        }
        // Altitudes: "1800", "2,500" (commas stripped by the caller).
        if n >= 500 && n % 100 == 0 {
            return altitude(n);
        }
        return digits(t);
    }
    t.to_string()
}

fn capitalize(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

/// `text` as the radio voice should say it.
pub fn spoken_form(text: &str) -> String {
    // "1,800" -> "1800" before splitting into words.
    let mut cleaned = String::with_capacity(text.len());
    let chars: Vec<char> = text.chars().collect();
    for (i, &c) in chars.iter().enumerate() {
        let comma_in_number = c == ','
            && i > 0
            && chars[i - 1].is_ascii_digit()
            && chars.get(i + 1).is_some_and(|n| n.is_ascii_digit());
        if !comma_in_number {
            cleaned.push(c);
        }
    }
    let mut out = Vec::new();
    let mut prev = "";
    for w in cleaned.split_whitespace() {
        out.push(word(w, prev));
        prev = w;
    }
    out.join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn callsigns_runways_frequencies_freeways_and_altitudes() {
        assert_eq!(
            spoken_form("Skyhawk 2SP, Van Nuys Tower, runway 16R cleared to land."),
            "Skyhawk Two Sierra Papa, Van Nuys Tower, runway one six right cleared to land."
        );
        assert_eq!(spoken_form("Contact tower 119.3"), "Contact tower one one niner point three");
        assert_eq!(spoken_form("At the 118, turn base"), "At the one-eighteen, turn base");
        assert_eq!(spoken_form("remain south of the 101"), "remain south of the one-oh-one");
        assert_eq!(spoken_form("follow the 405"), "follow the four-oh-five");
        assert_eq!(spoken_form("make a right 360 for spacing"), "make a right three-sixty for spacing");
        assert_eq!(spoken_form("maintain 1,800"), "maintain one thousand eight hundred");
        assert_eq!(spoken_form("climb and maintain 2500"), "climb and maintain two thousand five hundred");
        assert_eq!(spoken_form("turn left heading 230"), "turn left heading two three zero");
        assert_eq!(spoken_form("Cessna N172SP"), "Cessna November One Seven Two Sierra Papa");
        assert_eq!(spoken_form("runway 34"), "runway three four");
    }
}
